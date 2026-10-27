#include "status_report.h"
#include "scara_core_bridge.h"

#include <math.h>
#include "Arduino_PortentaMachineControl.h"
#include "project_config.h"
#include "robot_state.h"
#include "motion_service.h"
#include "kinematics.h"
#include "motor_map.h"

namespace {
float radToDeg(float r) {
  return r * 180.0f / PI;
}
}

void printPadStateIfNeeded(const PadState& st, bool ok) {
  if ((st.menuState != 0 && st.menuState != 2) || (millis() - g_lastPrintMs < PRINT_MS)) {
    return;
  }

  g_lastPrintMs = millis();

  Serial.print("I2C ok=");  Serial.print(ok ? 1 : 0);
  Serial.print(" | MENU="); Serial.print((unsigned)st.menuState);
  Serial.print(" | ESTOP=");Serial.print((unsigned)st.estop);
  Serial.print(" | MMODE="); Serial.print((unsigned)st.manualMode);
  Serial.print(" | SELSRC="); Serial.print((unsigned)st.selectedSource);
  Serial.print(" | misc=0x");
  if (st.misc < 16) Serial.print("0");
  Serial.print(st.misc, HEX);

  Serial.print(" | btn=0x");
  if (st.buttons < 0x1000) Serial.print("0");
  if (st.buttons < 0x0100) Serial.print("0");
  if (st.buttons < 0x0010) Serial.print("0");
  Serial.print(st.buttons, HEX);

  Serial.print(" | dpad=0x");
  if (st.dpad < 16) Serial.print("0");
  Serial.print(st.dpad, HEX);

  Serial.print(" | lx="); Serial.print((int)st.lx);
  Serial.print(" | ly="); Serial.print((int)st.ly);
  Serial.print(" | ry="); Serial.print((int)st.ry);
  Serial.print(" | rt="); Serial.println((unsigned)st.rt);
}

void resetTestDiPrinter() {
  di_menu4_initialized = false;
}

void handleMenuTestDI(const PadState& st) {
  g_lastManualMode = MANUAL_MODE_NONE;
  g_stop = true;
  g_cartesianControlEnabled = false;
  g_guiActive = false;

  if (!di_menu4_initialized) {
    di_menu4_initialized = true;

    Serial.println("=== MENU 4: TEST DI (CH00-CH06) ===");
    Serial.print("ESTOP="); Serial.println(st.estop ? 1 : 0);
    Serial.println("Estados iniciales:");

    for (int i = 0; i < DI_TOTAL; i++) {
      di_last[i] = MachineControl_DigitalInputs.read(DI_PINS[i]);
      Serial.print("CH0"); Serial.print(i);
      Serial.print(" = "); Serial.println(di_last[i]);
    }

    Serial.println("Se imprimira el switch solo cuando haya cambios.");
  }

  static uint32_t lastMs = 0;
  if (millis() - lastMs < 50) return;
  lastMs = millis();

  for (int i = 0; i < DI_TOTAL; i++) {
    int state = MachineControl_DigitalInputs.read(DI_PINS[i]);
    if (state != di_last[i]) {
      Serial.print("Cambio en CH0"); Serial.print(i);
      Serial.print(": ");
      Serial.print(di_last[i]);
      Serial.print(" -> ");
      Serial.println(state);

      di_last[i] = state;
    }
  }
}

void printCompactRobotStatus(Stream& out) {
  MotorMap::MotorTargets mNow;
  mNow.m1 = posSteps_M1;
  mNow.m2 = posSteps_M2;
  mNow.m3 = posSteps_M3;

  Kinematics::Joints qNow;
  bool haveJoints = MotorMap::motorToJoints(mNow, qNow, g_motorParams);

  float q1deg = NAN;
  float q2deg = NAN;
  float q3mm = NAN;
  float x = NAN;
  float y = NAN;
  float z = NAN;

  if (haveJoints) {
    Kinematics::Pose pNow = Kinematics::robotFK(qNow);
    q1deg = radToDeg(qNow.q1);
    q2deg = radToDeg(qNow.q2);
    q3mm = qNow.q3;
    x = pNow.x;
    y = pNow.y;
    z = pNow.z;
  }

  out.print("STATUS ");
  out.print("M1="); out.print(posSteps_M1);
  out.print(" M2="); out.print(posSteps_M2);
  out.print(" M3="); out.print(posSteps_M3);
  out.print(" X="); out.print(x, 3);
  out.print(" Y="); out.print(y, 3);
  out.print(" Z="); out.print(z, 3);
  out.print(" Q1="); out.print(q1deg, 3);
  out.print(" Q2="); out.print(q2deg, 3);
  out.print(" Q3="); out.print(q3mm, 3);
  out.print(" HOMED="); out.print(g_positionTrusted ? 1 : 0);
  out.print(" ESTOP="); out.print((g_estopActive || g_remoteEstop) ? 1 : 0);
  out.print(" ENA="); out.print((g_ena == ENA_LEVEL_ENABLE_CFG) ? 1 : 0);
  out.print(" BUSY="); out.print(isRobotBusy() ? 1 : 0);
  out.print(" SRC="); out.print((int)g_controlSource);
  out.print(" SEL="); out.print((int)g_panelSelectedSource);
  out.print(" STATE=");
  if (g_estopActive || g_remoteEstop) out.print((int)PMC_RUN_ERROR);
  else if (g_calibrating) out.print((int)PMC_RUN_HOMING);
  else if (g_sequenceActive) out.print((int)PMC_RUN_AUTO_RUNNING);
  else if (isRobotBusy()) out.print((int)PMC_RUN_BUSY);
  else if (g_hasError) out.print((int)PMC_RUN_ERROR);
  else out.print((int)PMC_RUN_READY);
  out.print(" ERR="); out.print(g_lastErrorText ? g_lastErrorText : "OK");
  out.println();
}
