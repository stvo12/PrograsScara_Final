#include "calibration_service.h"
#include "scara_core_bridge.h"

#include "project_config.h"
#include "robot_state.h"
#include "motion_service.h"
#include "esp_interface.h"
#include "belt_tracking.h"

namespace {
// Limites independientes del contador de pasos, que puede ser incorrecto
// precisamente cuando M3 ha perdido pasos.
bool seekM3EndstopBounded() {
  const long maxApproach = M3_MAX_STEPS_PHYS;
  long moved = 0;
  while (!endstopActive(DI_PINS[6]) && moved < maxApproach) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    const long chunk = (maxApproach - moved < HOMING_FAST_CHUNK)
        ? maxApproach - moved : HOMING_FAST_CHUNK;
    stepMotor3(false, chunk, g_PULSE3_US);
    moved += chunk;
    delay(HOMING_STEP_PAUSE_MS);
  }
  if (!endstopActive(DI_PINS[6])) {
    setRobotError("M3_ENDSTOP_NOT_FOUND");
    return false;
  }

  const long maxBackoff = m3DistanceMmToSteps(10.0f);
  moved = 0;
  while ((moved < HOMING_BACKOFF_STEPS || endstopActive(DI_PINS[6])) &&
         moved < maxBackoff) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    const long chunk = (maxBackoff - moved < HOMING_FAST_CHUNK)
        ? maxBackoff - moved : HOMING_FAST_CHUNK;
    stepMotor3(true, chunk, g_PULSE3_US);
    moved += chunk;
    delay(HOMING_STEP_PAUSE_MS);
  }
  if (endstopActive(DI_PINS[6])) {
    setRobotError("M3_ENDSTOP_STUCK");
    return false;
  }

  const long maxSlowApproach = moved + m3DistanceMmToSteps(1.0f);
  moved = 0;
  while (!endstopActive(DI_PINS[6]) && moved < maxSlowApproach) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    const long chunk = (maxSlowApproach - moved < HOMING_SLOW_CHUNK)
        ? maxSlowApproach - moved : HOMING_SLOW_CHUNK;
    stepMotor3(false, chunk, g_PULSE3_US);
    moved += chunk;
    delay(HOMING_STEP_PAUSE_MS);
  }
  if (!endstopActive(DI_PINS[6])) {
    setRobotError("M3_ENDSTOP_NOT_FOUND");
    return false;
  }
  return true;
}
}  // namespace

bool seekEndstopM1(int di_pin, bool movePositive) {
  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor1(movePositive, HOMING_FAST_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  for (long i = 0; i < HOMING_BACKOFF_STEPS; i += HOMING_FAST_CHUNK) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor1(!movePositive, HOMING_FAST_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor1(movePositive, HOMING_SLOW_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }
  return true;
}

bool seekEndstopM2(int di_pin, bool movePositive) {
  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor2(movePositive, HOMING_FAST_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  for (long i = 0; i < HOMING_BACKOFF_STEPS; i += HOMING_FAST_CHUNK) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor2(!movePositive, HOMING_FAST_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor2(movePositive, HOMING_SLOW_CHUNK, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }
  return true;
}

bool seekEndstopM3(int di_pin, bool movePositive) {
  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor3(movePositive, HOMING_FAST_CHUNK, g_PULSE3_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  for (long i = 0; i < HOMING_BACKOFF_STEPS; i += HOMING_FAST_CHUNK) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor3(!movePositive, HOMING_FAST_CHUNK, g_PULSE3_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  while (!endstopActive(di_pin)) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    stepMotor3(movePositive, HOMING_SLOW_CHUNK, g_PULSE3_US);
    delay(HOMING_STEP_PAUSE_MS);
  }

  return true;
}

bool runCalibrationSequence() {
  g_positionTrusted = false;
  g_armPositionTrusted = false;
  g_calibrationDone = false;
  g_calDone = false;
  g_stop = false;
  g_cartesianControlEnabled = false;
  g_calibrating = true;
  g_busy = true;
  clearTrackedPiece();
  clearRobotError();
  markControlSource(CONTROL_SOURCE_AUTO);
  enaWriteOnce(ENA_LEVEL_ENABLE_CFG);

  Serial.println("=== CALIBRACION: INICIO ===");

  // Primero elevar/referenciar Z para dejar la herramienta en una altura
  // segura antes de recorrer los limites articulares de M1 y M2.
  Serial.println("M3 -> buscando HOME en DI06...");
  posSteps_M3 = 0;

  // false hace que M3 busque el final de carrera superior DI06.
  if (!seekEndstopM3(DI_PINS[6], false)) goto CAL_ABORT;

  // Al tocar el switch definimos esa posicion como referencia fisica.
  posSteps_M3 = 0;
  Serial.println("M3 HOME OK. posSteps_M3 = 0");

  // Retirarse del switch hasta el HOME Z configurado.
  if (!moveM3ToSteps(M3_HOME_STEPS_PHYS)) goto CAL_ABORT;
  M3_homeSteps = M3_HOME_STEPS_PHYS;

  Serial.print("M3 -> yendo a HOME configurado: ");
  Serial.println(M3_HOME_STEPS_PHYS);
  Serial.print("M3 HOME FINAL OK. posSteps_M3=");
  Serial.println(posSteps_M3);

  Serial.println("M1 -> buscando MIN...");
  posSteps_M1 = 0;
  if (!seekEndstopM1(DI_PINS[3], false)) goto CAL_ABORT;

  posSteps_M1 = 0;
  M1_minSteps = 0;
  Serial.println("M1 MIN OK. posSteps_M1 = 0");

  Serial.println("M1 -> buscando MAX...");
  if (!seekEndstopM1(DI_PINS[2], true)) goto CAL_ABORT;

  M1_maxSteps = posSteps_M1;
  Serial.print("M1 MAX OK. maxSteps = ");
  Serial.println(M1_maxSteps);

  M1_homeSteps = M1_maxSteps / 2;
  Serial.print("M1 -> yendo a HOME (mitad): ");
  Serial.println(M1_homeSteps);
  if (!moveM1ToSteps(M1_homeSteps)) goto CAL_ABORT;
  Serial.print("M1 HOME OK. posSteps_M1=");
  Serial.println(posSteps_M1);

  Serial.println("M2 -> buscando MIN...");
  posSteps_M2 = 0;
  if (!seekEndstopM2(DI_PINS[1], false)) goto CAL_ABORT;

  posSteps_M2 = 0;
  M2_minSteps = 0;
  Serial.println("M2 MIN OK. posSteps_M2 = 0");

  Serial.println("M2 -> buscando MAX...");
  if (!seekEndstopM2(DI_PINS[0], true)) goto CAL_ABORT;

  M2_maxSteps = posSteps_M2;
  Serial.print("M2 MAX OK. maxSteps = ");
  Serial.println(M2_maxSteps);

  M2_homeSteps = M2_maxSteps / 4;
  Serial.print("M2 -> yendo a HOME (1/4): ");
  Serial.println(M2_homeSteps);
  if (!moveM2ToSteps(M2_homeSteps)) goto CAL_ABORT;
  Serial.print("M2 HOME OK. posSteps_M2=");
  Serial.println(posSteps_M2);

  g_calDone = true;
  g_calibrationDone = true;
  g_positionTrusted = true;
  g_armPositionTrusted = true;
  Serial.println("=== CALIBRACION: LISTA ===");
  Serial.print("M1 range [0.."); Serial.print(M1_maxSteps); Serial.println("]");
  Serial.print("M2 range [0.."); Serial.print(M2_maxSteps); Serial.println("]");
  Serial.print("M3 range [");   Serial.print(M3_MIN_STEPS_PHYS);
  Serial.print("..");           Serial.print(M3_MAX_STEPS_PHYS);
  Serial.println("]");

  applyCalibrationToCartesianControl();
  initCartesianPoseFromCurrentSteps();

  g_calibrating = false;
  g_busy = false;
  clearRobotError();
  return true;

CAL_ABORT:
  Serial.println("!!! CALIBRACION ABORTADA !!!");
  g_calibrating = false;
  g_busy = false;
  g_stop = true;
  if (g_estopActive || g_remoteEstop) {
    setRobotError("ESTOP_ACTIVE");
  } else {
    setRobotError("HOME_FAILED");
  }
  return false;
}

bool runM3CalibrationSequence() {
  // M1/M2 deben seguir referenciados; este procedimiento nunca los mueve.
  if (g_estopActive || g_remoteEstop) {
    setRobotError("M3_HOME_ESTOP_ACTIVE");
    return false;
  }
  if (isRobotBusy()) {
    setRobotError("M3_HOME_BUSY");
    return false;
  }
  if (!g_armPositionTrusted) {
    setRobotError("M3_HOME_ARM_UNTRUSTED");
    return false;
  }

  const long m1Before = posSteps_M1;
  const long m2Before = posSteps_M2;
  g_positionTrusted = false;
  g_calibrationDone = false;
  g_calDone = false;
  g_stop = false;
  g_cartesianControlEnabled = false;
  g_calibrating = true;
  g_busy = true;
  clearTrackedPiece();
  clearRobotError();
  markControlSource(CONTROL_SOURCE_AUTO);
  enaWriteOnce(ENA_LEVEL_ENABLE_CFG);
  Serial.println("=== CALIBRACION SOLO M3: INICIO ===");
  if (isBeltEncoderMoving()) {
    Serial.println("M3_HOME_BELT_ACTIVE: mantener libre la zona bajo la garra.");
  }

  if (!seekM3EndstopBounded()) goto M3_CAL_ABORT;
  posSteps_M3 = M3_MIN_STEPS_PHYS;
  if (!moveM3ToSteps(M3_HOME_STEPS_PHYS)) goto M3_CAL_ABORT;
  if (posSteps_M1 != m1Before || posSteps_M2 != m2Before) {
    g_armPositionTrusted = false;
    setRobotError("M3_HOME_ARM_MOVED");
    goto M3_CAL_ABORT;
  }
  if (!g_armPositionTrusted || posSteps_M3 != M3_HOME_STEPS_PHYS ||
      endstopActive(DI_PINS[6])) {
    setRobotError("M3_HOME_VERIFY_FAILED");
    goto M3_CAL_ABORT;
  }

  M3_homeSteps = M3_HOME_STEPS_PHYS;
  g_calDone = true;
  g_calibrationDone = true;
  g_positionTrusted = true;
  initCartesianPoseFromCurrentSteps();
  g_calibrating = false;
  g_busy = false;
  clearRobotError();
  Serial.print("=== CALIBRACION SOLO M3: LISTA. posSteps_M3=");
  Serial.println(posSteps_M3);
  return true;

M3_CAL_ABORT:
  if (posSteps_M1 != m1Before || posSteps_M2 != m2Before ||
      g_estopActive || g_remoteEstop) g_armPositionTrusted = false;
  g_calibrating = false;
  g_busy = false;
  g_stop = true;
  if (g_estopActive || g_remoteEstop) setRobotError("ESTOP_ACTIVE");
  else if (!g_hasError) setRobotError("M3_HOME_FAILED");
  Serial.print("!!! CALIBRACION SOLO M3 ABORTADA: ");
  Serial.println(g_lastErrorText);
  return false;
}

bool moveRobotToCalibratedHome() {
  if (!g_positionTrusted || g_estopActive || g_remoteEstop) return false;

  MotorMap::MotorTargets home;
  home.m1 = M1_homeSteps;
  home.m2 = M2_homeSteps;
  home.m3 = M3_homeSteps;

  g_homeTransitionActive = true;
  g_busy = true;
  g_stop = false;
  clearRobotError();
  enaWriteOnce(ENA_LEVEL_ENABLE_CFG);
  Serial.println("=== TRANSICION: REGRESANDO A HOME CALIBRADO ===");

  bool ok = moveMotorTargetsSync(
      home, selectAutoCoordinatedPulseUs(home, g_PULSE12_US));
  g_homeTransitionActive = false;
  g_busy = false;
  if (!ok) {
    g_stop = true;
    if (g_hasError) {
      Serial.print("RETURN_HOME_FAILED cause=");
      Serial.println(g_lastErrorText);
    } else {
      setRobotError("RETURN_HOME_FAILED");
      Serial.println("RETURN_HOME_FAILED cause=UNKNOWN");
    }
    return false;
  }

  initCartesianPoseFromCurrentSteps();
  Serial.println("=== TRANSICION: HOME ALCANZADO ===");
  return true;
}
