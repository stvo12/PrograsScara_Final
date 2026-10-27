#include "belt_tracking.h"
#include <math.h>

#include "project_config.h"
#include "robot_state.h"
#include "motor_map.h"
#include "kinematics.h"
#include "pick_reference.h"
#include <Arduino_PortentaMachineControl.h>
// =====================================================
// ESTADO INTERNO
// =====================================================

static TrackedPiece g_piece;
static TeachMapStatus g_teachMapStatus = TEACH_MAP_NO_PIECE;


// =====================================================
// ENCODER
// =====================================================

// Todavia no tienes encoder instalado.
// De momento esta variable queda en 0.
//
// Cuando llegue el encoder, esta variable sera
// actualizada mediante interrupciones.
static volatile int32_t g_encoderCount = 0;


// Velocidad estimada de la banda.
static float g_beltSpeedMmS = 0.0f;


// Variables necesarias para calcular velocidad.
static int32_t g_lastEncoderCount = 0;
static uint32_t g_lastEncoderTimeMs = 0;
static int32_t g_encoderRevolutions = 0;
static int32_t g_lastEncoderRevolutions = 0;
static uint32_t g_lastEncoderMotionMs = 0;
static bool g_encoderMoving = false;
static uint32_t g_lastEncoderDiagMs = 0;
static uint32_t g_lastEncoderZWarnMs = 0;
static int32_t g_encoderCountAtLastIndex = 0;
static bool g_encoderIndexBaselineValid = false;


// =====================================================
// INICIALIZACION
// =====================================================

void beltTrackingInit() {

  g_piece.active = false;

  g_encoderCount = 0;
  g_encoderRevolutions = 0;
  g_lastEncoderRevolutions = 0;

  g_beltSpeedMmS = 0.0f;

  g_lastEncoderCount = 0;
  g_lastEncoderTimeMs = millis();
  g_lastEncoderMotionMs = g_lastEncoderTimeMs;
  g_encoderMoving = false;
  g_lastEncoderDiagMs = 0;
  g_lastEncoderZWarnMs = 0;
  g_encoderCountAtLastIndex = 0;
  g_encoderIndexBaselineValid = false;

  if (BELT_ENCODER_ENABLED) {
    MachineControl_Encoders.setEncoding(
        BELT_ENCODER_CHANNEL,
        QEI::X2_ENCODING
    );
    MachineControl_Encoders.reset(BELT_ENCODER_CHANNEL);
    Serial.println("Encoder banda A0/B0/Z0 habilitado (X2).");
    Serial.print("P/R="); Serial.print(ENCODER_PULSES_PER_REV);
    Serial.print(" diametro rodillo [mm]="); Serial.print(ENCODER_ROLLER_DIAMETER_MM, 3);
    Serial.print(" mm/cuenta="); Serial.println(ENCODER_MM_PER_COUNT, 7);
  }
}


// =====================================================
// ACTUALIZAR ENCODER Y VELOCIDAD
// =====================================================

void updateBeltEstimator() {

  if (!BELT_ENCODER_ENABLED) {

    // Mientras no exista encoder,
    // dejamos velocidad cero.
    g_beltSpeedMmS = 0.0f;

    return;
  }

  uint32_t now = millis();

  int32_t rawCount = (int32_t)MachineControl_Encoders.getPulses(BELT_ENCODER_CHANNEL);
  g_encoderCount = rawCount * (int32_t)ENCODER_DIRECTION_SIGN;
  g_encoderRevolutions =
      (int32_t)MachineControl_Encoders.getRevolutions(BELT_ENCODER_CHANNEL);

  uint32_t deltaTimeMs =
      now - g_lastEncoderTimeMs;

  if (deltaTimeMs < 50) {
    return;
  }

  int32_t countNow =
      g_encoderCount;

  int32_t deltaCounts =
      countNow - g_lastEncoderCount;

  int32_t deltaIndexRevolutions =
      g_encoderRevolutions - g_lastEncoderRevolutions;

  // Los movimientos bloqueantes del brazo pueden impedir varias actualizaciones
  // consecutivas. En ese caso la muestra ya no esta cerca del flanco Z y no es
  // valida como referencia para comparar una vuelta completa.
  const bool encoderDiagGap = deltaTimeMs > 250;
  if (encoderDiagGap) {
    g_encoderIndexBaselineValid = false;
  }

  // Z entrega un pulso por vuelta. Se compara contra todos los pulsos A/B
  // acumulados desde el pulso Z anterior, no contra esta ventana de 50 ms.
  if (deltaIndexRevolutions != 0) {
    const bool singleIndexStep = labs(deltaIndexRevolutions) == 1;

    if (!encoderDiagGap && singleIndexStep && g_encoderIndexBaselineValid) {
      const int32_t observedCounts = labs(countNow - g_encoderCountAtLastIndex);
      const int32_t expectedCounts =
          labs(deltaIndexRevolutions) *
          (int32_t)ENCODER_PULSES_PER_REV *
          (int32_t)ENCODER_QUADRATURE_MULTIPLIER;
      const int32_t toleranceCounts = expectedCounts / 2;

      if ((observedCounts < expectedCounts - toleranceCounts ||
           observedCounts > expectedCounts + toleranceCounts) &&
          now - g_lastEncoderZWarnMs >= 1000) {
        g_lastEncoderZWarnMs = now;
        Serial.print("WARN ENCODER_Z_MISMATCH: A/B observados=");
        Serial.print(observedCounts);
        Serial.print(" esperados~=");
        Serial.println(expectedCounts);
      }
    }

    if (!encoderDiagGap && singleIndexStep) {
      g_encoderCountAtLastIndex = countNow;
      g_encoderIndexBaselineValid = true;
    } else {
      g_encoderIndexBaselineValid = false;
    }
  }

  float deltaMm =
      deltaCounts *
      ENCODER_MM_PER_COUNT;

  float deltaTimeS =
      deltaTimeMs / 1000.0f;

  float instantaneousSpeedMmS = deltaMm / deltaTimeS;
  const float SPEED_FILTER_ALPHA = 0.25f;

  if (deltaCounts != 0) {
    if (!g_encoderMoving) g_beltSpeedMmS = instantaneousSpeedMmS;
    else g_beltSpeedMmS +=
        SPEED_FILTER_ALPHA * (instantaneousSpeedMmS - g_beltSpeedMmS);
    g_lastEncoderMotionMs = now;
    g_encoderMoving = true;
  } else if (now - g_lastEncoderMotionMs >= 300) {
    g_beltSpeedMmS = 0.0f;
    g_encoderMoving = false;
  }

  g_lastEncoderCount =
      countNow;
  g_lastEncoderRevolutions = g_encoderRevolutions;

  g_lastEncoderTimeMs =
      now;

  if (g_encoderMoving && now - g_lastEncoderDiagMs >= 500) {
    g_lastEncoderDiagMs = now;
    Serial.print("ENCODER count="); Serial.print(g_encoderCount);
    Serial.print(" revZ="); Serial.print(g_encoderRevolutions);
    Serial.print(" mm="); Serial.print(getBeltTraveledMm(), 2);
    Serial.print(" speed_mm_s="); Serial.println(g_beltSpeedMmS, 2);
  }
}


// =====================================================
// REGISTRAR NUEVA PIEZA
// =====================================================

void registerDetectedPiece(
    uint8_t id,
    float cameraXmm,
    float cameraYmm,
    int16_t widthPx,
    int16_t heightPx
) {

  g_piece.active = true;

  g_piece.id = id;
  g_piece.cameraXmm = cameraXmm;
  g_piece.cameraYmm = cameraYmm;
  g_piece.widthPx = widthPx;
  g_piece.heightPx = heightPx;
  g_piece.orientation = PIECE_ORIENTATION_UNKNOWN;
  if (widthPx > 0 && heightPx > 0) {
    if (widthPx > heightPx) g_piece.orientation = PIECE_ORIENTATION_HORIZONTAL;
    else if (heightPx > widthPx) g_piece.orientation = PIECE_ORIENTATION_VERTICAL;
  }

  /*
   * IMPORTANTE:
   *
   * La homografia esta definida como:
   * cameraXmm = transversal, Tag 1 -> Tag 2 (0..424 mm)
   * cameraYmm = longitudinal, Tag 1 -> Tag 3 (0..248 mm)
   */

  if (!CAMERA_SWAP_AXES) {

    g_piece.initialLongitudinalMm =
        CAMERA_LONGITUDINAL_SIGN *
        cameraXmm;

    g_piece.transverseMm =
        CAMERA_TRANSVERSE_SIGN *
        cameraYmm;

  } else {

    g_piece.initialLongitudinalMm =
        CAMERA_LONGITUDINAL_SIGN *
        cameraYmm;

    g_piece.transverseMm =
        CAMERA_TRANSVERSE_SIGN *
        cameraXmm;
  }

  g_piece.encoderCountAtDetection =
      g_encoderCount;

  g_piece.detectionTimeMs =
      millis();
}


// =====================================================
// ESTADO DE PIEZA
// =====================================================

bool hasTrackedPiece() {
  return g_piece.active;
}

uint8_t getTrackedPieceId() {
  return g_piece.active ? g_piece.id : 0;
}

PieceOrientation getTrackedPieceOrientation() {
  return g_piece.active ? g_piece.orientation : PIECE_ORIENTATION_UNKNOWN;
}


void clearTrackedPiece() {
  g_piece.active = false;
  g_teachMapStatus = TEACH_MAP_NO_PIECE;
}


// =====================================================
// POSICION ACTUAL
// =====================================================

float getTrackedPieceCurrentLongitudinalMm() {

  if (!g_piece.active) {
    return 0.0f;
  }

  int32_t deltaCounts =
      g_encoderCount -
      g_piece.encoderCountAtDetection;

  float traveledMm =
      deltaCounts *
      ENCODER_MM_PER_COUNT;

  return
      g_piece.initialLongitudinalMm +
      traveledMm;
}


float getTrackedPieceTransverseMm() {

  if (!g_piece.active) {
    return 0.0f;
  }

  return g_piece.transverseMm;
}


// =====================================================
// VELOCIDAD
// =====================================================

float getBeltSpeedMmS() {
  return g_beltSpeedMmS;
}


// =====================================================
// PREDICCION
// =====================================================

float predictTrackedPieceLongitudinalMm(
    float futureTimeSeconds
) {

  float current =
      getTrackedPieceCurrentLongitudinalMm();

  return
      current +
      g_beltSpeedMmS *
      futureTimeSeconds;
}


// =====================================================
// BANDA -> COORDENADAS ROBOT
// =====================================================

static bool beltPointToRobot(
    float longitudinal,
    float transverse,
    float& robotX,
    float& robotY
) {
  if (!BELT_GEOMETRY_CALIBRATED ||
      !isfinite(longitudinal) ||
      !isfinite(transverse)) {
    return false;
  }

  const float localLongitudinal =
      longitudinal - CAMERA_TO_PICK_D_MM;

  const float localTransverse =
      transverse - BELT_TRANSVERSE_ZERO_MM;

  const float c = cosf(BELT_ANGLE_RAD);
  const float s = sinf(BELT_ANGLE_RAD);

  robotX =
      BELT_PICK_ORIGIN_ROBOT_X_MM +
      localLongitudinal * c -
      localTransverse * s;

  robotY =
      BELT_PICK_ORIGIN_ROBOT_Y_MM +
      localLongitudinal * s +
      localTransverse * c;

  return isfinite(robotX) && isfinite(robotY);
}

bool trackedPieceToRobot(
    float futureTimeSeconds,
    float& robotX,
    float& robotY
) {

  if (!g_piece.active) {
    return false;
  }

  /*
   * Posicion futura de la pieza medida
   * desde la referencia de la camara.
   */
  float predictedLongitudinal =
      predictTrackedPieceLongitudinalMm(
          futureTimeSeconds
      );

  return beltPointToRobot(
      predictedLongitudinal,
      g_piece.transverseMm,
      robotX,
      robotY
  );
}

bool trackedPiecePickTargetToRobot(
    float& robotX,
    float& robotY
) {
  if (!g_piece.active) {
    return false;
  }

  const bool ok = beltPointToRobot(
      CAMERA_TO_PICK_D_MM,
      g_piece.transverseMm,
      robotX,
      robotY
  );
  if (ok) {
    robotX += AUTO_PICK_X_CORRECTION_MM;
    robotY += AUTO_PICK_Y_CORRECTION_MM;
  }
  return ok;
}

// =====================================================
// MOCK CAMARA -> WORKSPACE ROBOT
// =====================================================

static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static long mapNormalizedToSteps(
    float t,
    long minSteps,
    long maxSteps
) {
  t = clamp01(t);

  return minSteps +
         lroundf(
           t * (float)(maxSteps - minSteps)
         );
}

bool trackedPieceToRobotMock(
    float& robotX,
    float& robotY,
    long& mockM1Steps,
    long& mockM2Steps
) {
  if (!g_piece.active) return false;

  // Necesitamos haber realizado HOME/calibración mecánica.
  if (!g_calibrationDone && !g_calDone) return false;

  if (M1_maxSteps <= 0 || M2_maxSteps <= 0) return false;

  // Ya comprobamos físicamente:
  //
  // longitudinal = Y cámara = 0..248 mm
  // transversal  = X cámara = 0..424 mm

  float longitudinal =
      getTrackedPieceCurrentLongitudinalMm();

  float transverse =
      getTrackedPieceTransverseMm();

  // Normalización de toda la zona calibrada.
  float u = longitudinal / 248.0f;
  float v = 1.0f - transverse / 424.0f;

  u = clamp01(u);
  v = clamp01(v);

  // ---------------------------------------------------
  // Zona mock segura alrededor del HOME
  // ---------------------------------------------------

  long m1SafeMin =
      M1_homeSteps - MOCK_M1_HALF_RANGE_STEPS;

  long m1SafeMax =
      M1_homeSteps + MOCK_M1_HALF_RANGE_STEPS;

  long m2SafeMin =
      M2_homeSteps - MOCK_M2_HALF_RANGE_STEPS;

  long m2SafeMax =
      M2_homeSteps + MOCK_M2_HALF_RANGE_STEPS;

  // Nunca permitir salir de los límites obtenidos
  // durante la calibración real.
  if (m1SafeMin < M1_minSteps) m1SafeMin = M1_minSteps;
  if (m1SafeMax > M1_maxSteps) m1SafeMax = M1_maxSteps;

  if (m2SafeMin < M2_minSteps) m2SafeMin = M2_minSteps;
  if (m2SafeMax > M2_maxSteps) m2SafeMax = M2_maxSteps;

  // Longitudinal cámara -> M1
  // Transversal cámara  -> M2
  mockM1Steps =
      mapNormalizedToSteps(
        u,
        m1SafeMin,
        m1SafeMax
      );

  mockM2Steps =
      mapNormalizedToSteps(
        v,
        m2SafeMin,
        m2SafeMax
      );

  // M3 no se toca.
  MotorMap::MotorTargets mMock;
  mMock.m1 = mockM1Steps;
  mMock.m2 = mockM2Steps;
  mMock.m3 = posSteps_M3;

  // Steps -> joints.
  Kinematics::Joints qMock;

  if (!MotorMap::motorToJoints(
      mMock,
      qMock,
      g_motorParams
  )) {
    return false;
  }

  // Joints -> pose cartesiana.
  Kinematics::Pose pMock = Kinematics::robotFK(qMock);

  if (!isfinite(pMock.x) ||
      !isfinite(pMock.y)) {
    return false;
  }

  robotX = pMock.x;
  robotY = pMock.y;

  return true;
}

uint32_t getTrackedPieceAgeMs() {

  if (!g_piece.active) {
    return 0;
  }

  return millis() - g_piece.detectionTimeMs;
}

bool trackedPieceToRobotTaught(
    float& robotX,
    float& robotY
) {
  if (!g_piece.active || !TEACH_MAP_ENABLED) {
    g_teachMapStatus = TEACH_MAP_NO_PIECE;
    return false;
  }

  const float cameraX = g_piece.cameraXmm;
  const float cameraY = g_piece.cameraYmm;

  if (!isfinite(cameraX) || !isfinite(cameraY)) {
    g_teachMapStatus = TEACH_MAP_INVALID_COORDS;
    return false;
  }
  if (cameraX < TEACH_CAM_X_MIN_MM) {
    g_teachMapStatus = TEACH_MAP_OUT_LEFT;
    return false;
  }
  if (cameraX > TEACH_CAM_X_MAX_MM) {
    g_teachMapStatus = TEACH_MAP_OUT_RIGHT;
    return false;
  }
  if (cameraY < TEACH_CAM_Y_MIN_MM || cameraY > TEACH_CAM_Y_MAX_MM) {
    g_teachMapStatus = TEACH_MAP_OUT_LONGITUDINAL;
    return false;
  }

  robotX =
      TEACH_ROBOT_X_FROM_CAM_X * cameraX +
      TEACH_ROBOT_X_FROM_CAM_Y * cameraY +
      TEACH_ROBOT_X_OFFSET;

  robotY =
      TEACH_ROBOT_Y_FROM_CAM_X * cameraX +
      TEACH_ROBOT_Y_FROM_CAM_Y * cameraY +
      TEACH_ROBOT_Y_OFFSET;

  if (!isfinite(robotX) || !isfinite(robotY)) {
    g_teachMapStatus = TEACH_MAP_INVALID_COORDS;
    return false;
  }

  g_teachMapStatus = TEACH_MAP_OK;
  return true;
}

TeachMapStatus getTeachMapStatus() {
  return g_teachMapStatus;
}

const char* teachMapStatusText(TeachMapStatus status) {
  switch (status) {
    case TEACH_MAP_OK: return "TEACH_OK";
    case TEACH_MAP_NO_PIECE: return "TEACH_NO_PIECE";
    case TEACH_MAP_INVALID_COORDS: return "TEACH_INVALID_COORDS";
    case TEACH_MAP_OUT_LEFT: return "TEACH_OUT_LEFT_PHYSICAL_LIMIT";
    case TEACH_MAP_OUT_RIGHT: return "TEACH_OUT_RIGHT";
    case TEACH_MAP_OUT_LONGITUDINAL: return "TEACH_OUT_CAMERA_Y";
    default: return "TEACH_UNKNOWN";
  }
}

int32_t getBeltEncoderCount() {
  return g_encoderCount;
}

int32_t getBeltEncoderRevolutions() {
  return g_encoderRevolutions;
}

float getBeltTraveledMm() {
  return g_encoderCount * ENCODER_MM_PER_COUNT;
}

bool isBeltEncoderMoving() {
  return g_encoderMoving;
}

float getPickLongitudinalTargetMm() {
  if (TEACH_MAP_ENABLED || MOCK_VISION_ENABLED ||
      !AUTO_PICK_TIMING_FOLLOWS_XY_CORRECTION) return CAMERA_TO_PICK_D_MM;
  return PickReference::longitudinal(CAMERA_TO_PICK_D_MM,
      AUTO_PICK_X_CORRECTION_MM, AUTO_PICK_Y_CORRECTION_MM,
      cosf(BELT_ANGLE_RAD), sinf(BELT_ANGLE_RAD));
}

float getTrackedPieceDistanceToPickMm() {
  if (!g_piece.active) return NAN;

  return getPickLongitudinalTargetMm() -
         getTrackedPieceCurrentLongitudinalMm();
}
