#include "command_dispatcher.h"
#include "scara_core_bridge.h"
#include "project_config.h"

#include "robot_state.h"
#include "motion_service.h"
#include "calibration_service.h"
#include "status_report.h"
#include "gui_protocol.h"
#include "sequence_runner.h"
#include "esp_interface.h"
#include "belt_tracking.h"
#include "motor_map.h"
#include "kinematics.h"
#include "rotation_compensation.h"

#include <string.h>
#include <Wire.h>

namespace {
bool s_visualCalibRequested = false;
bool s_m3CalibAttempted = false;
uint32_t s_nextM3CalibAttemptMs = 0;
bool s_resetAutoStateAfterCalibration = false;
uint32_t s_lastVisualPollMs = 0;
CalibrationMode s_activeCalibrationMode = CALIBRATION_MODE_NONE;
float s_lastAutoRotationDeg = 40.0f;
uint32_t s_compensatedRotationCommandMs = 0;
bool s_descentCloseSent = false;
uint32_t s_descentCloseSentMs = 0;

float horizontalCloseHeightAbovePickMm() {
  const float stepsPerMm = m3ConfiguredStepsPerMm();
  if (!isfinite(stepsPerMm) || stepsPerMm <= 0.0f) return NAN;
  return M3_MIN_MM + (float)(posSteps_M3 - M3_MIN_STEPS_PHYS) / stepsPerMm
      - AUTO_PICK_Z_MM;
}

bool closeHorizontalDuringDescent() {
  if (s_descentCloseSent) return true;
  updateBeltEstimator();
  const float distanceMm = getTrackedPieceDistanceToPickMm();
  if (!isfinite(distanceMm) || !isBeltEncoderMoving()) {
    setRobotError("AUTO_DESCENT_TRACKING_LOST");
    return false;
  }
  if (distanceMm < -AUTO_PICK_CAPTURE_MAX_LATE_MM) {
    setRobotError("AUTO_CLOSE_MISSED_DURING_DESCENT");
    return false;
  }
  const float abovePickMm = horizontalCloseHeightAbovePickMm();
  if (!isfinite(abovePickMm)) {
    setRobotError("AUTO_CLOSE_HEIGHT_INVALID");
    return false;
  }
  // Ambas condiciones deben cumplirse: ventana de pieza y altura de captura.
  if (distanceMm > AUTO_PICK_CLOSE_LEAD_MM ||
      abovePickMm > AUTO_PICK_CLOSE_MAX_ABOVE_HORIZONTAL_MM) return true;
  if (!sendGripperAngleI2C(GRIPPER_CLOSED_ANGLE)) {
    setRobotError("AUTO_CLOSE_I2C_FAILED");
    return false;
  }
  s_descentCloseSent = true;
  s_descentCloseSentMs = millis();
  Serial.print("GRIPPER_CLOSE_DURING_DESCENT distance_mm="); Serial.print(distanceMm, 1);
  Serial.print(" M3_steps="); Serial.print(posSteps_M3);
  Serial.print(" above_pick_mm="); Serial.print(abovePickMm, 1);
  Serial.print(" cmd="); Serial.println(GRIPPER_CLOSED_ANGLE);
  return true;
}

// Solo recupera una ventana de captura perdida. No libera paros ni convierte
// errores de motores, finales de carrera o comunicacion en abortos normales.
bool recoverMissedCaptureFromCurrentSteps() {
  if (!g_positionTrusted || !g_calibrationDone || g_stop ||
      g_estopActive || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) {
    return false;
  }

  MotorMap::MotorTargets current;
  current.m1 = posSteps_M1;
  current.m2 = posSteps_M2;
  current.m3 = posSteps_M3;
  Kinematics::Joints joints;
  if (!MotorMap::withinMotorLimits(current, g_motorLimits) ||
      !MotorMap::motorToJoints(current, joints, g_motorParams)) {
    setRobotError("AUTO_ABORT_POSE_INVALID");
    return false;
  }
  const Kinematics::Pose pose = Kinematics::robotFK(joints);
  if (Safety::validatePose(pose, g_safetyCfg) != Safety::SafetyCode::OK) {
    setRobotError("AUTO_ABORT_POSE_INVALID");
    return false;
  }

  // moveToPoseXYZ no actualiza la pose al abortar: usar los pasos realmente
  // emitidos, no el destino Z=pick que quedo incompleto.
  ScaraPose::init(g_poseState, pose);
  g_lastControlMs = millis();
  clearRobotError();
  Serial.print("AUTO_CAPTURE_MISSED pose_z_mm="); Serial.println(pose.z, 1);

  float retractZ = AUTO_PICK_Z_MM + AUTO_PICK_HOVER_ABOVE_MM;
  if (retractZ < pose.z) retractZ = pose.z;
  // Retirar solo Z: conservar el XY real del aborto y las protecciones normales.
  if (!moveToPoseXYZ(pose.x, pose.y, retractZ)) {
    Serial.println("AUTO_ABORT_RETRACT_FAILED");
    return false;
  }
  Serial.println("AUTO_ABORT_RETRACT_OK: calibracion conservada.");
  return true;
}

void resetCalibrationSession() {
  s_visualCalibRequested = false;
  s_m3CalibAttempted = false;
  s_nextM3CalibAttemptMs = 0;
  s_lastVisualPollMs = 0;
  s_activeCalibrationMode = CALIBRATION_MODE_NONE;
  if (!g_calibrating) g_busy = false;
}

bool buildPickPlaceParams(uint8_t pieceType, float pickX, float pickY,
                          PickPlaceParams& params) {
  params = PickPlaceParams();
  params.pickX = pickX;
  params.pickY = pickY;
  params.pickZ = AUTO_PICK_Z_MM;
  params.safeZ = PLACE_TRAVEL_Z_MM;
  params.graspAngle = GRIPPER_CLOSED_ANGLE;

  switch (pieceType) {
    case 1:
      params.placeX = PLACE_PIECE1_X_MM;
      params.placeY = PLACE_PIECE1_Y_MM;
      params.placeZ = PLACE_PIECE1_Z_MM;
      return true;
    case 2:
      params.placeX = PLACE_PIECE2_X_MM;
      params.placeY = PLACE_PIECE2_Y_MM;
      params.placeZ = PLACE_PIECE2_Z_MM;
      return true;
    case 5:
      params.placeX = PLACE_PIECE5_X_MM;
      params.placeY = PLACE_PIECE5_Y_MM;
      params.placeZ = PLACE_PIECE5_Z_MM;
      return true;
    default:
      return false;
  }
}

uint8_t pieceTypeFromHuskyName(const char* name) {
  // Auto solo recoge Pieza5; las otras descargas se conservan para uso explicito.
  if (strcmp(name, "Pieza") == 0) return 5;
  return 0;
}
}

ManualMode getManualMode(const PadState& st) {
  switch (st.manualMode) {
    case 1: return MANUAL_MODE_JOINT;
    case 2: return MANUAL_MODE_CARTESIAN;
    default: return MANUAL_MODE_NONE;
  }
}

void handleMenuCalibration(const PadState& st) {
  g_lastManualMode = MANUAL_MODE_NONE;
  g_guiActive = false;

  if (st.estop || g_remoteEstop) {
    activateEmergencyStop();
    resetCalibrationSession();
    return;
  }

  CalibrationMode requested = (CalibrationMode)st.calibrationMode;
  if (requested < CALIBRATION_MODE_FULL || requested > CALIBRATION_MODE_M3) return;

  if (requested != s_activeCalibrationMode) {
    s_activeCalibrationMode = requested;
    s_resetAutoStateAfterCalibration = true;
    s_visualCalibRequested = false;
    s_m3CalibAttempted = false;
    s_nextM3CalibAttemptMs = 0;
    s_lastVisualPollMs = 0;
    if (requested != CALIBRATION_MODE_M3) g_visualCalibrationDone = false;
    clearRobotError();

    if (requested == CALIBRATION_MODE_FULL || requested == CALIBRATION_MODE_ARM) {
      g_positionTrusted = false;
      g_armPositionTrusted = false;
      g_calibrationDone = false;
      g_calDone = false;
    }

    if (requested == CALIBRATION_MODE_CAMERA && g_positionTrusted) {
      if (!moveRobotToCalibratedHome()) return;
    }
  }

  if (requested == CALIBRATION_MODE_M3) {
    if (!s_m3CalibAttempted &&
        (int32_t)(millis() - s_nextM3CalibAttemptMs) >= 0) {
      const bool ok = runM3CalibrationSequence();
      const bool startRejected = !ok &&
          (strcmp(g_lastErrorText, "M3_HOME_ESTOP_ACTIVE") == 0 ||
           strcmp(g_lastErrorText, "M3_HOME_BUSY") == 0 ||
           strcmp(g_lastErrorText, "M3_HOME_ARM_UNTRUSTED") == 0);
      // Rechazar el inicio no consume la seleccion. Tras un fallo durante
      // homing, exigir salir y volver a seleccionar Solo M3, no repetir solo.
      s_m3CalibAttempted = !startRejected;
      s_nextM3CalibAttemptMs = millis() + 1000;
      if (!ok) {
        Serial.print(startRejected ? "M3_HOME_BLOCKED: " : "M3_HOME_FAILED: ");
        Serial.println(g_lastErrorText);
      }
    }
    return;
  }

  if ((requested == CALIBRATION_MODE_FULL || requested == CALIBRATION_MODE_ARM) && !g_positionTrusted) {
    if (!runCalibrationSequence()) {
      setRobotError("HOME_FAILED");
      s_visualCalibRequested = false;
      return;
    }
    if (requested == CALIBRATION_MODE_ARM) {
      Serial.println("Calibracion solo brazo completada.");
      return;
    }
  }

  if ((requested == CALIBRATION_MODE_FULL || requested == CALIBRATION_MODE_CAMERA) &&
      !s_visualCalibRequested && !g_visualCalibrationDone) {
    Serial.println("Iniciando calibracion visual Husky...");

    if (startHuskyTagCalibrationI2C()) {
      Serial.println("Comando calibracion visual enviado a ESP Husky 0x29.");
      s_visualCalibRequested = true;
      s_lastVisualPollMs = 0;
      g_busy = true;
    } else {
      Serial.println("No se pudo enviar comando de calibracion visual a ESP Husky 0x29.");
      setRobotError("HUSKY_CAL_CMD_FAIL");
      return;
    }
  }

  if (!s_visualCalibRequested) {
    return;
  }

  if (millis() - s_lastVisualPollMs < 500) {
    return;
  }

  s_lastVisualPollMs = millis();

  HuskyCalibState hs;
  if (readHuskyCalibrationStateI2C(hs)) {
    Serial.print("Husky calib flags = 0x");
    Serial.println(hs.flags, HEX);

    if (hs.flags & 0x80) {
      g_visualCalibrationDone = true;
      g_busy = false;
      Serial.println("Calibracion visual completada.");
      Serial.print("camXMin=");
      Serial.print(hs.camXMin);
      Serial.print(" camXMax=");
      Serial.print(hs.camXMax);
      Serial.print(" camYMin=");
      Serial.print(hs.camYMin);
      Serial.print(" camYMax=");
      Serial.println(hs.camYMax);
    } else if (hs.flags & 0x40) {
      g_busy = false;
      setRobotError("HUSKY_CAL_FAILED");
    }
  } else {
    Serial.println("No se pudo leer estado de calibracion visual.");
  }
}

void handleMenuManual(const PadState& st) {
  if (st.estop || g_remoteEstop) {
    activateEmergencyStop();
    return;
  }

  if (!g_positionTrusted) {
    Serial.println("MANUAL BLOCKED: POSITION UNTRUSTED");
    g_stop = true;
    g_cartesianControlEnabled = false;
    g_lastManualMode = MANUAL_MODE_NONE;
    g_guiActive = false;
    setRobotError("POSITION_UNTRUSTED");
    return;
  }

  if (st.selectedSource == PANEL_SOURCE_PC) {
    g_stop = false;
    g_cartesianControlEnabled = false;
    g_lastManualMode = MANUAL_MODE_NONE;
    g_guiActive = true;
    markControlSource(CONTROL_SOURCE_GUI);
    if (!g_hasError || g_lastErrorText == nullptr || strcmp(g_lastErrorText, "ESTOP_ACTIVE") == 0) {
      clearRobotError();
    }
    return;
  }

  g_guiActive = false;

  ManualMode mode = getManualMode(st);

  if (mode != g_lastManualMode) {
    if (mode == MANUAL_MODE_CARTESIAN) {
      initCartesianPoseFromCurrentSteps();
    }
    if ((mode == MANUAL_MODE_JOINT || mode == MANUAL_MODE_CARTESIAN) && g_ena != ENA_LEVEL_ENABLE_CFG) {
      enaWriteOnce(ENA_LEVEL_ENABLE_CFG);
    }
    g_lastManualMode = mode;
  }

  switch (mode) {
    case MANUAL_MODE_JOINT:
      clearRobotError();
      handleManualJoint(st);
      break;

    case MANUAL_MODE_CARTESIAN:
      clearRobotError();
      handleManualCartesian(st);
      break;

    case MANUAL_MODE_NONE:
    default:
      g_stop = true;
      g_cartesianControlEnabled = false;
      setRobotError("MANUAL_MODE_NONE");
      break;
  }
}

void handleMenuTeach(const PadState& st) {
  static bool circleWasPressed = false;
  static const uint16_t BTN_CIRCLE = 0x0002;

  if (st.estop || g_remoteEstop) {
    activateEmergencyStop();
    circleWasPressed = false;
    return;
  }

  if (!g_positionTrusted) {
    g_stop = true;
    setRobotError("POSITION_UNTRUSTED");
    circleWasPressed = false;
    return;
  }

  g_lastManualMode = MANUAL_MODE_JOINT;
  handleManualJoint(st);

  bool circlePressed = (st.buttons & BTN_CIRCLE) != 0;
  if (circlePressed && !circleWasPressed) {
    Serial.println();
    Serial.println("========== TEACH POINT ==========");
    if (st.huskyDetected && st.huskyMapped) {
      Serial.print("CAM_NOMBRE="); Serial.print(st.huskyName);
      Serial.print(" CAM_PIXEL_X="); Serial.print(st.huskyX);
      Serial.print(" CAM_PIXEL_Y="); Serial.print(st.huskyY);
      Serial.print(" CAM_X_MM="); Serial.print(st.huskyXmm10 / 10.0f, 1);
      Serial.print(" CAM_Y_MM="); Serial.println(st.huskyYmm10 / 10.0f, 1);
    } else {
      Serial.println("CAMERA=INVALID (pieza no detectada o mapeo no valido)");
    }
    printCompactRobotStatus(Serial);
    Serial.println("=================================");
  }
  circleWasPressed = circlePressed;
}

void handleMenuAuto(const PadState& st) {

  static uint32_t lastPrintMs = 0;

  /*
   * Evita registrar la misma pieza muchas veces
   * mientras permanece visible en la cámara.
   */
  static bool detectionLatched = false;

  static bool mockMoveAttempted = false;
  static bool finalDescentDone = false;
  static float prepositionRobotX = 0.0f;
  static float prepositionRobotY = 0.0f;
  static bool pickLineCrossingReported = false;

  static bool progressWatchActive = false;
  static int32_t lastProgressEncoderCount = 0;
  static uint32_t lastProgressMs = 0;

  // La calibracion descarta el seguimiento y mueve Z: no reutilizar una
  // preposicion/retirada pendiente del intento anterior al volver a Auto.
  if (s_resetAutoStateAfterCalibration) {
    detectionLatched = false;
    mockMoveAttempted = false;
    finalDescentDone = false;
    pickLineCrossingReported = false;
    progressWatchActive = false;
    s_descentCloseSent = false;
    s_descentCloseSentMs = 0;
    s_resetAutoStateAfterCalibration = false;
  }

  /*
   * Cuando la cámara deja de verla,
   * permitimos registrar la siguiente.
   */
  if (!st.huskyDetected) {
    detectionLatched = false;
    if (!hasTrackedPiece()) {
      mockMoveAttempted = false;
    }
  }

  if (st.estop || g_remoteEstop) {
    activateEmergencyStop();
    return;
  }

  if (!g_positionTrusted) {
    g_stop = true;
    setRobotError("POSITION_UNTRUSTED");
    return;
  }

  g_lastManualMode = MANUAL_MODE_NONE;
  g_cartesianControlEnabled = false;
  g_guiActive = false;

  // AUTO necesita liberar el paro lógico para poder mover.
  // El ESTOP real/remoto se valida aparte.
  g_stop = false;

  // Mientras se transporta una pieza, otra deteccion no puede abrir ni
  // girar la garra. La siguiente se atiende despues de terminar la secuencia.
  if (isSequenceRunning()) {
    if (st.huskyDetected) detectionLatched = true;
    return;
  }

  // Si se descarto una pieza con M3 abajo, retirar la pinza antes de
  // aceptar otra deteccion o dejar que la banda siga pasando piezas.
  if (!hasTrackedPiece() && finalDescentDone) {
    if (!moveToPoseXYZ(prepositionRobotX, prepositionRobotY,
                       AUTO_PICK_Z_MM + AUTO_PICK_HOVER_ABOVE_MM)) {
      Serial.println("AUTO_ABORT_RETRACT_FAILED");
      g_positionTrusted = false;
      g_calibrationDone = false;
      g_calDone = false;
      g_stop = true;
      return;
    }
    finalDescentDone = false;
    Serial.println("AUTO_ABORT_RETRACT_OK");
  }

  clearRobotError();

  markControlSource(
      CONTROL_SOURCE_AUTO
  );


  // ===================================================
  // 1. VALIDAR HUSKYLENS
  // ===================================================

  if (st.huskyFlags & 0x10) {

    if (millis() - lastPrintMs >= 500) {

      lastPrintMs = millis();

      Serial.println(
        "AUTO: error leyendo HuskyLens."
      );
    }

    return;
  }


  if (!(st.huskyFlags & 0x01)) {

    if (millis() - lastPrintMs >= 500) {

      lastPrintMs = millis();

      Serial.println(
        "AUTO: HuskyLens no inicializada."
      );
    }

    return;
  }


  if (!(st.huskyFlags & 0x02)) {

    if (millis() - lastPrintMs >= 500) {

      lastPrintMs = millis();

      Serial.println(
        "AUTO: ModeloPruebaV2 no seleccionado."
      );
    }

    return;
  }

  // No se borra por edad ni al salir de camara. Mientras el encoder progrese,
  // la pieza conserva todo el tiempo que necesite para llegar al pick.
  if (hasTrackedPiece()) {
    const int32_t encoderNow = getBeltEncoderCount();
    if (!progressWatchActive) {
      progressWatchActive = true;
      lastProgressEncoderCount = encoderNow;
      lastProgressMs = millis();
    }

    if (labs((long)encoderNow - (long)lastProgressEncoderCount) >=
        AUTO_TRACK_PROGRESS_MIN_COUNTS) {
      lastProgressEncoderCount = encoderNow;
      lastProgressMs = millis();
    }

    if (millis() - lastProgressMs >= AUTO_TRACK_NO_PROGRESS_TIMEOUT_MS) {
      Serial.println("AUTO: tracker liberado: encoder sin progreso.");
      clearTrackedPiece();
      mockMoveAttempted = false;
      progressWatchActive = false;
    }
  } else {
    progressWatchActive = false;
    pickLineCrossingReported = false;
  }

  if (!MOCK_MOVE_ENABLED &&
      !AUTO_PICK_ENABLED &&
      hasTrackedPiece()) {
    const float distanceRemainingMm = getTrackedPieceDistanceToPickMm();

    if (distanceRemainingMm <= 0.0f && !pickLineCrossingReported) {
      pickLineCrossingReported = true;
      Serial.print("DEBUG: pieza cruzo linea de pick; distancia [mm] = ");
      Serial.println(distanceRemainingMm, 1);
    }

    if (distanceRemainingMm < -AUTO_PICK_MISSED_DISTANCE_MM) {
      Serial.println("DEBUG: pieza paso ventana de pick; tracker liberado.");
      clearTrackedPiece();
      progressWatchActive = false;
      pickLineCrossingReported = false;
    }
  }
  // ===================================================
  // 2. REGISTRAR NUEVA PIEZA
  // ===================================================

  if (st.huskyDetected &&
      st.huskyMapped &&
      !detectionLatched) {

    /*
     * Primera versión:
     *
     * Solo seguimos una pieza simultáneamente.
     *
     * Más adelante podemos convertirlo en cola
     * de varias piezas.
     */

    if (!hasTrackedPiece()) {

      // El ID de la HuskyLens puede cambiar al actualizar/entrenar el modelo.
      // Solo nombres conocidos pueden iniciar un pick automatico.
      const uint8_t pieceType = pieceTypeFromHuskyName(st.huskyName);
      if (!(st.huskyFlags & 0x40) || pieceType == 0) {
        Serial.print("AUTO_NOMBRE_DESCONOCIDO='");
        Serial.print(st.huskyName);
        Serial.println("'; pieza ignorada.");
        detectionLatched = true;
        return;
      }

      const bool liveAutoPick = AUTO_PICK_ENABLED &&
          !TEACH_MAP_ENABLED && !MOCK_VISION_ENABLED;
      if (liveAutoPick && (st.huskyW <= 0 || st.huskyH <= 0 ||
                           st.huskyW == st.huskyH)) {
        Serial.print("AUTO_ORIENTATION_UNKNOWN W=");
        Serial.print(st.huskyW);
        Serial.print(" H=");
        Serial.println(st.huskyH);
        detectionLatched = true;
        return;
      }

      float cameraXmm =
          st.huskyXmm10 / 10.0f;

      float cameraYmm =
          st.huskyYmm10 / 10.0f;

      registerDetectedPiece(
          pieceType,
          cameraXmm,
          cameraYmm,
          st.huskyW,
          st.huskyH
      );
      mockMoveAttempted = false;
      finalDescentDone = false;
      progressWatchActive = false;
      s_descentCloseSent = false;
      pickLineCrossingReported = false;

      // En diagnostico sin movimiento no actuar tampoco la pinza.
      if (MOCK_MOVE_ENABLED || AUTO_PICK_ENABLED) {
        if (!openGripperI2C()) {
          Serial.println("GRIPPER_OPEN_I2C_FAILED");
          clearTrackedPiece();
          detectionLatched = true;
          return;
        }
      }

      if (liveAutoPick && !AUTO_ROTATION_COMPENSATION_ENABLED) {
        const PieceOrientation orientation = getTrackedPieceOrientation();
        const uint8_t rotationAngle =
            orientation == PIECE_ORIENTATION_HORIZONTAL
                ? AUTO_ROTATION_HORIZONTAL_ANGLE
                : AUTO_ROTATION_VERTICAL_ANGLE;
        if (!sendRotationAngleI2C(rotationAngle)) {
          Serial.println("AUTO_ROTATION_I2C_FAILED");
          clearTrackedPiece();
          detectionLatched = true;
          return;
        }
        s_lastAutoRotationDeg = rotationAngle;
        Serial.print("AUTO_ORIENTATION=");
        Serial.print(orientation == PIECE_ORIENTATION_HORIZONTAL
                         ? "HORIZONTAL" : "VERTICAL");
        Serial.print(" W="); Serial.print(st.huskyW);
        Serial.print(" H="); Serial.print(st.huskyH);
        Serial.print(" ROTATION_CMD="); Serial.println(rotationAngle);
      }

      Serial.println();
      Serial.println(
        "========== NUEVA PIEZA =========="
      );

      Serial.print("Nombre = ");
      Serial.println(st.huskyName);
      Serial.print("Tipo interno = ");
      Serial.println(pieceType);

      Serial.print(
        "Pixel = ("
      );
      Serial.print(
        st.huskyX
      );
      Serial.print(", ");
      Serial.print(
        st.huskyY
      );
      Serial.println(")");

      Serial.print(
        "Camara [mm] = ("
      );
      Serial.print(
        cameraXmm,
        1
      );
      Serial.print(", ");
      Serial.print(
        cameraYmm,
        1
      );
      Serial.println(")");

      Serial.println(
        "Pieza registrada para seguimiento."
      );

      Serial.println(
        "================================="
      );
    }

    detectionLatched = true;
  }


  // ===================================================
  // 3. SI NO HAY PIEZA, TERMINAMOS
  // ===================================================

  if (!hasTrackedPiece()) {

    if (millis() - lastPrintMs >= 500) {

      lastPrintMs = millis();

      Serial.println(
        "AUTO: esperando pieza..."
      );
    }

    return;
  }


  // ===================================================
  // 4. POSICION ACTUAL
  // ===================================================

  float currentLongitudinal =
      getTrackedPieceCurrentLongitudinalMm();

  float transverse =
      getTrackedPieceTransverseMm();

  float beltSpeed =
      getBeltSpeedMmS();

  // Evitar que el volcado serial de 500 ms retrase el instante de descenso.
  const float trackedDescentEstimateS =
      getTrackedPieceOrientation() == PIECE_ORIENTATION_HORIZONTAL
          ? AUTO_FINAL_DESCENT_HORIZONTAL_ESTIMATE_S : AUTO_FINAL_DESCENT_ESTIMATE_S;
  const bool nearFinalDescent = mockMoveAttempted &&
      !TEACH_MAP_ENABLED && !MOCK_VISION_ENABLED &&
      getTrackedPieceDistanceToPickMm() <=
          fabsf(beltSpeed) * (trackedDescentEstimateS + 0.15f);


  // ===================================================
  // 6. CONVERTIR A SISTEMA DEL ROBOT
  // ===================================================

  float robotX = 0.0f;
  float robotY = 0.0f;

  long mockM1Steps = 0;
  long mockM2Steps = 0;

  bool transformOK = false;

  if (TEACH_MAP_ENABLED) {

    transformOK =
        trackedPieceToRobotTaught(
          robotX,
          robotY
        );

  } else if (MOCK_VISION_ENABLED) {

    transformOK =
        trackedPieceToRobotMock(
          robotX,
          robotY,
          mockM1Steps,
          mockM2Steps
        );

  } else {

    transformOK =
        trackedPiecePickTargetToRobot(
          robotX,
          robotY
        );
  }


  // ===================================================
  // 7. DEBUG CADA 500 ms
  // ===================================================

  if (!nearFinalDescent && millis() - lastPrintMs >= 500) {

    lastPrintMs = millis();

    Serial.println();
    Serial.println(
      "---------- AUTO TRACK ----------"
    );

    Serial.print(
      "Pos actual banda [mm] = "
    );
    Serial.println(
      currentLongitudinal,
      1
    );

    Serial.print(
      "Pos transversal [mm] = "
    );
    Serial.println(
      transverse,
      1
    );

    Serial.print(
      "Velocidad banda [mm/s] = "
    );
      Serial.println(
        beltSpeed,
        1
      );

      Serial.print("Encoder count=");
      Serial.print(getBeltEncoderCount());
      Serial.print(" revZ=");
      Serial.print(getBeltEncoderRevolutions());
      Serial.print(" recorrido [mm]=");
      Serial.print(getBeltTraveledMm(), 2);
      Serial.print(" moving=");
      Serial.println(isBeltEncoderMoving() ? 1 : 0);

      Serial.print("Distancia restante a pick [mm] = ");
      Serial.println(getTrackedPieceDistanceToPickMm(), 1);

    if (transformOK) {

    Serial.print("Objetivo robot XY = (");
    Serial.print(robotX, 1);
    Serial.print(", ");
    Serial.print(robotY, 1);
    Serial.println(")");

    if (TEACH_MAP_ENABLED) {
      Serial.println("Mapa TEACH camara->robot activo.");
    } else if (MOCK_VISION_ENABLED) {
      Serial.print("MOCK steps M1=");
      Serial.print(mockM1Steps);

      Serial.print(" M2=");
      Serial.println(mockM2Steps);
    } else {
      Serial.println("Mapa rigido CAD camara->robot activo.");
    }

  } else {

    if (TEACH_MAP_ENABLED) {
      Serial.println(
        teachMapStatusText(getTeachMapStatus())
      );
    } else if (MOCK_VISION_ENABLED) {
      Serial.println(
        "Transformacion MOCK camara->robot invalida."
      );
    } else {
      Serial.println(
        "Transformacion banda->robot invalida."
      );
    }
  }


    if (!BELT_ENCODER_ENABLED) {

      Serial.println(
        "ENCODER: pendiente."
      );

      Serial.println(
        "Velocidad y desplazamiento = 0."
      );
    }


    if (TEACH_MAP_ENABLED || MOCK_VISION_ENABLED) {

      if (!MOCK_MOVE_ENABLED) {
        Serial.println(
          "MOVIMIENTO MOCK BLOQUEADO."
        );
      }

    } else if (!AUTO_PICK_ENABLED) {

      Serial.println(
        "MOVIMIENTO AUTO BLOQUEADO."
      );
    }

    Serial.println(
      "--------------------------------"
    );
  }

  // ===================================================
  // SECUENCIA DE PICK CON LA TRANSFORMACION SELECCIONADA
  // ===================================================

  if (!transformOK) {
    Serial.print("AUTO descarta pieza: ");
    if (TEACH_MAP_ENABLED) {
      Serial.println(teachMapStatusText(getTeachMapStatus()));
    } else {
      Serial.println("transformacion camara->robot invalida.");
    }
    clearTrackedPiece();
    mockMoveAttempted = false;
    return;
  }

  const bool movementEnabled =
      (TEACH_MAP_ENABLED || MOCK_VISION_ENABLED)
          ? MOCK_MOVE_ENABLED
          : AUTO_PICK_ENABLED;

  // Primera etapa: calcular e imprimir sin mover.
  if (!movementEnabled) {
    return;
  }

  // El robot debe estar calibrado.
  if (!g_calibrationDone && !g_calDone) {
    Serial.println(
      "AUTO bloqueado: robot no calibrado."
    );
    return;
  }

  if (isRobotBusy()) {
    return;
  }

  const bool insidePickArea =
      robotX >= PICK_AREA_X_MIN_MM &&
      robotX <= PICK_AREA_X_MAX_MM &&
      robotY >= PICK_AREA_Y_MIN_MM &&
      robotY <= PICK_AREA_Y_MAX_MM;

  if (!insidePickArea) {
    Serial.println("AUTO_TARGET_OUTSIDE_PICK_AREA");
    clearTrackedPiece();
    mockMoveAttempted = false;
    return;
  }

    // ---------------------------------------------------
    // M1/M2 se alinean mientras M3 baja hasta la altura de espera.
    // El ultimo descenso se sincroniza despues con el encoder.
    // ---------------------------------------------------

    const bool liveAutoPick = !TEACH_MAP_ENABLED && !MOCK_VISION_ENABLED;
    const float hoverZ = AUTO_PICK_Z_MM + AUTO_PICK_HOVER_ABOVE_MM;

    if (!mockMoveAttempted) {
      if (!AUTO_PICK_DESCENT_ENABLED) {
        Serial.println("PICK_DESCENT_BLOCKED: falta validar Z y pinza.");
        clearTrackedPiece();
        mockMoveAttempted = false;
        return;
      }

      mockMoveAttempted = true;

      Serial.println();
      Serial.println("========== AUTO PREPOSITION ==========");
      Serial.print("Objetivo XY = (");
      Serial.print(robotX, 1);
      Serial.print(", ");
      Serial.print(robotY, 1);
      Serial.println(")");
      Serial.print("Descenso coordinado a Z = ");
      Serial.println(liveAutoPick ? hoverZ : AUTO_PICK_Z_MM, 1);
      if (liveAutoPick) {
        Serial.print("PICK_REFERENCE nominal_mm="); Serial.print(CAMERA_TO_PICK_D_MM, 1);
        Serial.print(" effective_mm="); Serial.print(getPickLongitudinalTargetMm(), 1);
        Serial.print(" follows_xy=");
        Serial.println(AUTO_PICK_TIMING_FOLLOWS_XY_CORRECTION ? 1 : 0);
      }

      if (liveAutoPick && AUTO_ROTATION_COMPENSATION_ENABLED) {
        const PieceOrientation orientation = getTrackedPieceOrientation();
        Kinematics::Pose rotationPose;
        rotationPose.x = robotX;
        rotationPose.y = robotY;
        rotationPose.z = hoverZ;
        Kinematics::Joints rotationJoints;
        MotorMap::MotorTargets rotationTargets;
        uint8_t rotationAngle = 0;
        if (!isfinite(AUTO_ROTATION_S0_DEG) ||
            !isfinite(AUTO_ROTATION_GAIN) || fabsf(AUTO_ROTATION_GAIN) < 0.001f) {
          Serial.println("AUTO_ROTATION_CALIBRATION_REQUIRED: configurar S0 y GAIN.");
          clearTrackedPiece();
          mockMoveAttempted = false;
          return;
        }
        if (orientation == PIECE_ORIENTATION_UNKNOWN ||
            !solveIKNearestCurrent(rotationPose, rotationJoints, rotationTargets)) {
          Serial.println("AUTO_ROTATION_TARGET_INVALID");
          clearTrackedPiece();
          mockMoveAttempted = false;
          return;
        }
        const float linkDeg = (rotationJoints.q1 + rotationJoints.q2) *
                              (180.0f / 3.14159265358979323846f);
        // Ancho>alto: Yg || Yc, por lo tanto Xg perpendicular a Yc.
        // Alto>ancho: Xg || Yc. Ambas condiciones son modulo 180 grados.
        const float desiredXDeg = AUTO_ROTATION_CAMERA_Y_ROBOT_DEG -
            (orientation == PIECE_ORIENTATION_HORIZONTAL ? 90.0f : 0.0f);
        if (!solveCompensatedRotation(linkDeg, desiredXDeg,
                AUTO_ROTATION_S0_DEG, AUTO_ROTATION_GAIN,
                AUTO_ROTATION_ZERO_X_OFFSET_DEG,
                AUTO_ROTATION_MIN_ANGLE, AUTO_ROTATION_MAX_ANGLE,
                s_lastAutoRotationDeg, rotationAngle)) {
          Serial.println("AUTO_ROTATION_UNREACHABLE: orientacion fuera del rango del servo.");
          clearTrackedPiece();
          mockMoveAttempted = false;
          return;
        }
        // Enviar antes de la preposicion permite girar mientras M1/M2 se mueven.
        // moveToPoseXYZ resolvera la misma rama IK: los motores aun no cambiaron.
        if (!sendRotationAngleI2C(rotationAngle)) {
          Serial.println("AUTO_ROTATION_I2C_FAILED");
          clearTrackedPiece();
          mockMoveAttempted = false;
          return;
        }
        s_lastAutoRotationDeg = rotationAngle;
        s_compensatedRotationCommandMs = millis();
        Serial.print("AUTO_ROTATION_COMPENSATED orientation=");
        Serial.print(orientation == PIECE_ORIENTATION_HORIZONTAL
                         ? "HORIZONTAL" : "VERTICAL");
        Serial.print(" link_deg="); Serial.print(linkDeg, 2);
        Serial.print(" desired_x_deg="); Serial.print(desiredXDeg, 2);
        Serial.print(" servo_cmd="); Serial.println(rotationAngle);
      }

      if (!moveToPoseXYZ(robotX, robotY,
                         liveAutoPick ? hoverZ : AUTO_PICK_Z_MM)) {
        Serial.println("AUTO_PREPOSITION_FAILED");
        clearTrackedPiece();
        mockMoveAttempted = false;
        return;
      }

      prepositionRobotX = robotX;
      prepositionRobotY = robotY;
      Serial.println("AUTO_PREPOSITION_OK");
      return;
    }

    // ---------------------------------------------------
    // Bajar desde la altura de espera hasta Z de pick antes del cruce,
    // segun la velocidad de la banda y el tiempo estimado de M3. El cierre
    // sigue gobernado por el cruce del encoder.
    // ---------------------------------------------------
    float distanceRemainingMm = getTrackedPieceDistanceToPickMm();
    float speedMmS = getBeltSpeedMmS();

    if (liveAutoPick && finalDescentDone &&
        distanceRemainingMm < -AUTO_PICK_CAPTURE_MAX_LATE_MM) {
      Serial.print("AUTO_PICK_MISSED_AFTER_DESCENT distance_mm=");
      Serial.println(distanceRemainingMm, 1);
      clearTrackedPiece();
      mockMoveAttempted = false;
      return;
    }

    if (distanceRemainingMm < -AUTO_PICK_MISSED_DISTANCE_MM) {
      Serial.println("AUTO_PICK_MISSED: pieza ya paso la ventana.");
      clearTrackedPiece();
      mockMoveAttempted = false;
      return;
    }

    if (!isBeltEncoderMoving() || speedMmS < AUTO_PICK_MIN_SPEED_MM_S) {
      if (liveAutoPick && finalDescentDone) {
        Serial.println("AUTO_PICK_ABORT: banda detenida durante el descenso.");
        clearTrackedPiece();
        mockMoveAttempted = false;
      }
      return;
    }

    if (liveAutoPick && !finalDescentDone) {
      if (distanceRemainingMm <= 0.0f) {
        Serial.println("AUTO_PICK_MISSED_BEFORE_DESCENT");
        clearTrackedPiece();
        mockMoveAttempted = false;
        return;
      }

      // El giro normalmente se asienta durante la preposicion. Si esta fue
      // corta, completar ese tiempo antes de bajar a la pieza.
      if (AUTO_ROTATION_COMPENSATION_ENABLED &&
          millis() - s_compensatedRotationCommandMs < GRIPPER_SETTLE_MS) return;

      const bool horizontal =
          getTrackedPieceOrientation() == PIECE_ORIENTATION_HORIZONTAL;
      const float descentEstimateS = horizontal
          ? AUTO_FINAL_DESCENT_HORIZONTAL_ESTIMATE_S
          : AUTO_FINAL_DESCENT_ESTIMATE_S;
      const float leadMm = speedMmS * descentEstimateS;
      if (distanceRemainingMm > leadMm) return;

      Serial.println("========== AUTO FINAL DESCENT ==========");
      Serial.print("distance_before_mm="); Serial.print(distanceRemainingMm, 1);
      Serial.print(" lead_mm="); Serial.print(leadMm, 1);
      Serial.print(" speed_mm_s="); Serial.println(speedMmS, 1);
      const uint32_t descentStartMs = millis();
      if (!moveToPoseXYZ(robotX, robotY, AUTO_PICK_Z_MM,
                        horizontal ? closeHorizontalDuringDescent : nullptr)) {
        const bool missedCapture = horizontal && !s_descentCloseSent &&
            g_hasError && strcmp(g_lastErrorText,
                                "AUTO_CLOSE_MISSED_DURING_DESCENT") == 0;
        clearTrackedPiece();
        mockMoveAttempted = false;
        if (missedCapture && recoverMissedCaptureFromCurrentSteps()) {
          finalDescentDone = false;
          s_descentCloseSent = false;
          s_descentCloseSentMs = 0;
          return;
        }
        Serial.println("AUTO_FINAL_DESCENT_FAILED");
        g_positionTrusted = false;
        g_calibrationDone = false;
        g_calDone = false;
        g_stop = true;
        return;
      }

      finalDescentDone = true;
      updateBeltEstimator();
      distanceRemainingMm = getTrackedPieceDistanceToPickMm();
      speedMmS = getBeltSpeedMmS();
      Serial.print("AUTO_FINAL_DESCENT_DONE elapsed_ms=");
      Serial.print(millis() - descentStartMs);
      Serial.print(" distance_after_mm=");
      Serial.println(distanceRemainingMm, 1);

      if (!s_descentCloseSent &&
          distanceRemainingMm < -AUTO_PICK_CAPTURE_MAX_LATE_MM) {
        Serial.println("AUTO_PICK_MISSED_AFTER_DESCENT");
        clearTrackedPiece();
        mockMoveAttempted = false;
        return;
      }
    }

    const float closeLeadMm = liveAutoPick
        ? (getTrackedPieceOrientation() == PIECE_ORIENTATION_VERTICAL
               ? AUTO_PICK_CLOSE_LEAD_VERTICAL_MM
               : AUTO_PICK_CLOSE_LEAD_MM)
        : 0.0f;
    if (!s_descentCloseSent && distanceRemainingMm > closeLeadMm) {
      return;
    }

    // La ruta posterior al descenso tampoco debe saltarse el limite de altura.
    if (liveAutoPick && !s_descentCloseSent &&
        getTrackedPieceOrientation() == PIECE_ORIENTATION_HORIZONTAL) {
      const float abovePickMm = horizontalCloseHeightAbovePickMm();
      if (!isfinite(abovePickMm)) {
        setRobotError("AUTO_CLOSE_HEIGHT_INVALID");
        clearTrackedPiece();
        mockMoveAttempted = false;
        return;
      }
      if (abovePickMm > AUTO_PICK_CLOSE_MAX_ABOVE_HORIZONTAL_MM) return;
    }

    Serial.println("========== PICK CLOSE TRIGGER ==========");
    Serial.print("distance_mm="); Serial.print(distanceRemainingMm, 1);
    Serial.print(" close_lead_mm="); Serial.print(closeLeadMm, 1);
    Serial.print(" speed_mm_s="); Serial.println(speedMmS, 1);

    const uint8_t pieceType = getTrackedPieceId();
    PickPlaceParams params;
    if (!buildPickPlaceParams(pieceType, robotX, robotY, params)) {
      Serial.print("AUTO_PIECE_TYPE_INVALID=");
      Serial.println(pieceType);
      clearTrackedPiece();
      mockMoveAttempted = false;
      return;
    }

    if (!startPrepositionedPickPlaceSequence(params, s_descentCloseSent,
                                            s_descentCloseSentMs)) {
      Serial.println("AUTO_PICKPLACE_START_FAILED");
      clearTrackedPiece();
      mockMoveAttempted = false;
      return;
    }

    // La secuencia de pick & place pasa a controlar M3.
    finalDescentDone = false;

    Serial.print("AUTO PIEZA=");
    Serial.print(pieceType);
    Serial.print(" GRIP_ANGLE=");
    Serial.print(params.graspAngle);
    Serial.print(" PLACE=(");
    Serial.print(params.placeX, 3);
    Serial.print(", ");
    Serial.print(params.placeY, 3);
    Serial.print(", ");
    Serial.print(params.placeZ, 3);
    Serial.println(")");

  clearTrackedPiece();
  mockMoveAttempted = false;
  return;
}

void dispatchPadState(const PadState& st) {
  static RobotMode lastOperationalMode = ROBOT_MODE_IDLE;
  if (st.menuState != 1 && s_activeCalibrationMode != CALIBRATION_MODE_NONE) {
    resetCalibrationSession();
  }
  RobotMode requestedMode = ROBOT_MODE_IDLE;
  if (st.menuState == 2) requestedMode = ROBOT_MODE_MANUAL;
  if (st.menuState == 3) requestedMode = ROBOT_MODE_AUTO;
  if (st.menuState == 6) requestedMode = ROBOT_MODE_TEACH;

  if ((requestedMode == ROBOT_MODE_MANUAL || requestedMode == ROBOT_MODE_AUTO ||
       requestedMode == ROBOT_MODE_TEACH) &&
      requestedMode != lastOperationalMode) {
    if (!g_positionTrusted) {
      g_stop = true;
      setRobotError("POSITION_UNTRUSTED");
      lastOperationalMode = requestedMode;
      return;
    }
    if (!moveRobotToCalibratedHome()) {
      lastOperationalMode = requestedMode;
      return;
    }
  }
  if (requestedMode != ROBOT_MODE_IDLE) lastOperationalMode = requestedMode;

  switch (st.menuState) {
    case 1:
      resetTestDiPrinter();
      handleMenuCalibration(st);
      break;
    case 2:
      resetTestDiPrinter();
      handleMenuManual(st);
      break;
    case 3:
      resetTestDiPrinter();
      handleMenuAuto(st);
      break;
    case 4:
      handleMenuTestDI(st);
      break;
    case 6:
      resetTestDiPrinter();
      handleMenuTeach(st);
      break;
    default:
      resetTestDiPrinter();
      g_lastManualMode = MANUAL_MODE_NONE;
      g_guiActive = false;
      g_stop = true;
      g_cartesianControlEnabled = false;
      setRobotError("IDLE_STOP");
      break;
  }
}

void dispatchPendingGuiCommands() {
  if (!guiHasPendingCommand()) return;

  GuiCommand cmd = guiTakePendingCommand();
  if (!cmd.pending) return;

  if (cmd.type != GUI_CMD_GET_STATUS && g_panelSelectedSource != PANEL_SOURCE_PC) {
    setRobotError("SOURCE_NOT_PC");
    Serial.println("ERR SOURCE_NOT_PC");
    return;
  }

  markControlSource(CONTROL_SOURCE_GUI);
  g_guiActive = true;
  g_stop = false;

  switch (cmd.type) {
    case GUI_CMD_GET_STATUS:
      printCompactRobotStatus(Serial);
      break;

    case GUI_CMD_HOME:
      if (runCalibrationSequence()) {
        clearRobotError();
        Serial.println("OK");
      } else {
        setRobotError("HOME_FAILED");
        Serial.println("ERR HOME_FAILED");
      }
      break;

    case GUI_CMD_HOME_M3:
      if (runM3CalibrationSequence()) {
        Serial.println("OK HOME_M3");
      } else {
        Serial.print("ERR HOME_M3 ");
        Serial.println(g_lastErrorText);
      }
      break;

    case GUI_CMD_ENABLE:
      if (!g_positionTrusted || g_estopActive || g_remoteEstop) {
        setRobotError("POSITION_UNTRUSTED");
        Serial.println("ERR POSITION_UNTRUSTED");
      } else {
        enaWriteOnce(ENA_LEVEL_ENABLE_CFG);
        clearRobotError();
        Serial.println("OK");
      }
      break;

    case GUI_CMD_DISABLE:
      g_stop = true;
      setRobotError("USE_ESTOP_TO_DISABLE");
      Serial.println("ERR USE_ESTOP_TO_DISABLE");
      break;

    case GUI_CMD_ESTOP:
      g_remoteEstop = true;
      activateEmergencyStop();
      Serial.println("OK");
      break;

    case GUI_CMD_RESET_ESTOP:
      g_remoteEstop = false;
      releaseEmergencyStop();
      Serial.println("OK");
      break;

    case GUI_CMD_GOTO_XYZ:
      if (moveToPoseXYZ(cmd.a, cmd.b, cmd.c)) {
        clearRobotError();
        Serial.println("OK");
      } else {
        if (!g_hasError) setRobotError("GOTO_FAILED");
        Serial.println("ERR GOTO_FAILED");
      }
      break;

    case GUI_CMD_MOVE_JOINTS:
      if (moveToJoints(cmd.a, cmd.b, cmd.c)) {
        clearRobotError();
        Serial.println("OK");
      } else {
        if (!g_hasError) setRobotError("MOVEJ_FAILED");
        Serial.println("ERR MOVEJ_FAILED");
      }
      break;

    case GUI_CMD_RUN_PICK: {
      const uint8_t pieceType = (uint8_t)cmd.c;
      PickPlaceParams params;
      const bool paramsOK = buildPickPlaceParams(pieceType, cmd.a, cmd.b, params);

      if (!paramsOK) {
        setRobotError("INVALID_PIECE_ID");
        Serial.println("ERR INVALID_PIECE_ID");
      } else if (!g_positionTrusted || !g_calibrationDone) {
        setRobotError("POSITION_UNTRUSTED");
        Serial.println("ERR POSITION_UNTRUSTED: calibre el brazo primero");
      } else if (g_estopActive || g_remoteEstop) {
        setRobotError("ESTOP_ACTIVE");
        Serial.println("ERR ESTOP_ACTIVE");
      } else if (startPickPlaceSequence(params)) {
        clearRobotError();
        Serial.print("OK RUN_PICK X=");
        Serial.print(cmd.a, 1);
        Serial.print(" Y=");
        Serial.print(cmd.b, 1);
        Serial.print(" PICK_Z=");
        Serial.print(AUTO_PICK_Z_MM, 1);
        Serial.print(" SAFE_Z=");
        Serial.print(PLACE_TRAVEL_Z_MM, 1);
        Serial.print(" PIEZA=");
        Serial.print(pieceType);
        Serial.print(" GRIP_ANGLE=");
        Serial.print(params.graspAngle);
        Serial.print(" PLACE=(");
        Serial.print(params.placeX, 3);
        Serial.print(", ");
        Serial.print(params.placeY, 3);
        Serial.print(", ");
        Serial.print(params.placeZ, 3);
        Serial.println(")");
      } else {
        setRobotError("PICK_BUSY");
        Serial.println("ERR PICK_BUSY");
      }
      break;
    }

    case GUI_CMD_RUN_PICKPLACE: {
      PickPlaceParams params;
      params.pickX = 150.0f;
      params.pickY = 50.0f;
      params.placeX = 100.0f;
      params.placeY = 150.0f;
      params.safeZ = DEFAULT_SAFE_Z_MM;
      params.pickZ = DEFAULT_PICK_Z_MM;
      params.placeZ = DEFAULT_PLACE_Z_MM;
      params.graspAngle = GRIPPER_CLOSED_ANGLE;
      if (startPickPlaceSequence(params)) {
        clearRobotError();
        Serial.println("OK");
      } else {
        setRobotError("PICKPLACE_BUSY");
        Serial.println("ERR PICKPLACE_BUSY");
      }
      break;
    }

    case GUI_CMD_NONE:
    default:
      break;
  }
}
