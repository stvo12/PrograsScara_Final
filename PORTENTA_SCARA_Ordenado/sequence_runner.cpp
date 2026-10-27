#include "sequence_runner.h"
#include "scara_core_bridge.h"

#include "motion_service.h"
#include "calibration_service.h"
#include "robot_state.h"
#include "esp_interface.h"
#include "project_config.h"

namespace {
PickPlaceParams g_params;
bool g_running = false;
bool g_pickOnly = false;
uint8_t g_step = 0;
bool g_closeAlreadySent = false;
uint32_t g_closeSentMs = 0;

void finishSequence(const char* message) {
  g_running = false;
  g_sequenceActive = false;
  g_busy = false;
  clearRobotError();
  Serial.println(message);
}
}

void sequenceRunnerInit() {
  g_running = false;
  g_pickOnly = false;
  g_step = 0;
  g_sequenceActive = false;
  g_closeAlreadySent = false;
}

bool startPickSequence(float pickX, float pickY, float pickZ, float safeZ,
                       uint8_t graspAngle) {
  if (g_running) return false;

  (void)graspAngle;

  g_params = PickPlaceParams();
  g_params.pickX = pickX;
  g_params.pickY = pickY;
  g_params.pickZ = pickZ;
  g_params.safeZ = safeZ;
  g_params.graspAngle = GRIPPER_CLOSED_ANGLE;
  g_pickOnly = true;
  g_running = true;
  g_step = 0;
  g_sequenceActive = true;
  g_closeAlreadySent = false;
  g_busy = true;
  clearRobotError();
  markControlSource(CONTROL_SOURCE_AUTO);
  Serial.println("PICK_START");
  return true;
}

bool startPickPlaceSequence(const PickPlaceParams& params) {
  if (g_running) return false;
  g_params = params;
  g_params.graspAngle = GRIPPER_CLOSED_ANGLE;
  g_pickOnly = false;
  g_running = true;
  g_step = 0;
  g_sequenceActive = true;
  g_closeAlreadySent = false;
  g_busy = true;
  clearRobotError();
  markControlSource(CONTROL_SOURCE_AUTO);
  Serial.println("PICKPLACE_START");
  return true;
}

bool startPrepositionedPickPlaceSequence(const PickPlaceParams& params,
                                        bool closeAlreadySent, uint32_t closeSentMs) {
  if (g_running) return false;

  g_params = params;
  g_params.graspAngle = GRIPPER_CLOSED_ANGLE;
  g_pickOnly = false;
  g_running = true;
  g_closeAlreadySent = closeAlreadySent;
  g_closeSentMs = closeSentMs;
  // AUTO ya abrio y oriento la pinza, y bajo de hover a Z de recogida.
  // Comenzar directamente con el cierre evita retrasar el intercepto.
  g_step = 2;
  g_sequenceActive = true;
  g_busy = true;
  clearRobotError();
  markControlSource(CONTROL_SOURCE_AUTO);
  Serial.println("AUTO_PICKPLACE_START");
  return true;
}

bool isSequenceRunning() {
  return g_running;
}

void updateSequenceRunner() {
  if (!g_running) return;
  if (g_stop || g_remoteEstop) {
    g_running = false;
    g_sequenceActive = false;
    g_busy = false;
    setRobotError("PICKPLACE_ABORT");
    Serial.println("PICKPLACE_ABORT");
    return;
  }

  bool ok = false;
  switch (g_step) {
    case 0:
      // La pinza debe estar abierta antes de aproximarse a la pieza.
      Serial.print("GRIPPER_OPEN_CMD=");
      Serial.println(GRIPPER_OPEN_ANGLE);
      ok = openGripperI2C();
      if (ok) delay(GRIPPER_SETTLE_MS);
      break;
    case 1:
      // X/Y se alinean mientras M3 desciende en el mismo movimiento coordinado.
      ok = moveToPoseXYZ(g_params.pickX, g_params.pickY, g_params.pickZ);
      break;
    case 2:
      if (g_closeAlreadySent) {
        // Cierre enviado durante descenso: conservar la espera total configurada
        // desde esa orden, sin reenviarla ni reiniciar la espera.
        const uint32_t elapsedMs = millis() - g_closeSentMs;
        if (elapsedMs < GRIPPER_CLOSE_SETTLE_MS)
          delay(GRIPPER_CLOSE_SETTLE_MS - elapsedMs);
        ok = true;
        g_closeAlreadySent = false;
        break;
      }
      // El ESP recibe el angulo final en una sola orden. Esperamos a que el
      // servo termine fisicamente de cerrar antes de retirar M3.
      Serial.print("GRIPPER_CLOSE_CMD=");
      Serial.println(g_params.graspAngle);
      ok = sendGripperAngleI2C(g_params.graspAngle);
      if (ok) delay(GRIPPER_CLOSE_SETTLE_MS);
      break;
    case 3:
      ok = moveToPoseXYZ(g_params.pickX, g_params.pickY, g_params.safeZ);
      break;
    case 4:
      ok = moveToPoseXYZ(g_params.placeX, g_params.placeY, g_params.safeZ);
      break;
    case 5:
      ok = moveToPoseXYZ(g_params.placeX, g_params.placeY, g_params.placeZ);
      break;
    case 6:
      Serial.print("GRIPPER_RELEASE_CMD=");
      Serial.println(GRIPPER_OPEN_ANGLE);
      ok = openGripperI2C();
      if (ok) delay(GRIPPER_SETTLE_MS);
      break;
    case 7:
      ok = moveToPoseXYZ(g_params.placeX, g_params.placeY, g_params.safeZ);
      break;
    case 8:
      // Finaliza siempre en el HOME real obtenido durante la calibracion.
      ok = moveRobotToCalibratedHome();
      break;
    default:
      ok = true;
      break;
  }

  if (!ok) {
    if (!g_hasError) setRobotError("PICKPLACE_STEP_FAILED");
    Serial.print("PICKPLACE_STEP_FAILED: ");
    Serial.println(g_lastErrorText);
    g_running = false;
    g_sequenceActive = false;
    g_busy = false;
    return;
  }

  g_step++;
  if (g_pickOnly && g_step > 3) {
    finishSequence("PICK_DONE");
    return;
  }

  if (g_step > 8) {
    finishSequence("PICKPLACE_DONE");
  }
}
