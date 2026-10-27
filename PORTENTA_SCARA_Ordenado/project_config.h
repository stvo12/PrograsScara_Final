#pragma once

#include <Arduino.h>
#include <Arduino_PortentaMachineControl.h>

// ============================================================================
// Configuración editable declarada en el .ino
// Estas variables se definen en PORTENTA_SCARA_I2C_CONTROL.ino para que
// puedas seguir editándolas fácil desde un solo lugar.
// ============================================================================

// --- Comunicación ---
extern const int SERIAL_BAUDRATE;
extern const uint8_t ESP32_ADDR;
extern const uint8_t HUSKY_ESP32_ADDR;
extern const uint32_t I2C_FREQ;
extern const uint8_t ESTOP_CONFIRM_SAMPLES;
extern const uint32_t PRINT_MS;
extern const uint32_t LOOP_DELAY_MS;

// --- Salidas digitales / canales ---
extern const uint8_t STEP1_CH_CFG;
extern const uint8_t DIR1_CH_CFG;
extern const uint8_t STEP2_CH_CFG;
extern const uint8_t DIR2_CH_CFG;
extern const uint8_t STEP3_CH_CFG;
extern const uint8_t DIR3_CH_CFG;
extern const uint8_t ENA_CH_CFG;

// --- Lógica de drivers ---
extern const PinStatus ENA_LEVEL_ENABLE_CFG;
extern const PinStatus ENA_LEVEL_DISABLE_CFG;
extern const PinStatus DIR1_LEVEL_NEG_CFG;
extern const PinStatus DIR2_LEVEL_NEG_CFG;
extern const PinStatus DIR3_LEVEL_NEG_CFG;

// --- Digital Inputs ---
extern const int ENDSTOP_ACTIVE_LEVEL;
extern const int DI_TOTAL;
extern const int DI_PINS[];

// --- Geometria y transmision de M2 ---
extern const float SCARA_L1_MM;
extern const float SCARA_L2_MM;
extern const uint16_t M2_MOTOR_PULLEY_TEETH;
extern const uint16_t M2_JOINT_PULLEY_TEETH;

// --- Transmision mecanica del eje Z ---
extern const float M3_STEPPER_STEPS_PER_REV;
extern const uint16_t M3_STAGE1_DRIVER_TEETH;
extern const uint16_t M3_STAGE1_DRIVEN_TEETH;
extern const uint16_t M3_STAGE2_DRIVER_TEETH;
extern const uint16_t M3_STAGE2_DRIVEN_TEETH;
extern const float M3_SCREW_LEAD_MM_PER_REV;

inline float m3ConfiguredStepsPerMm() {
  if (M3_STAGE1_DRIVER_TEETH == 0 || M3_STAGE2_DRIVER_TEETH == 0 ||
      M3_SCREW_LEAD_MM_PER_REV <= 0.0f) {
    return 0.0f;
  }

  const float motorRevPerScrewRev =
      ((float)M3_STAGE1_DRIVEN_TEETH / (float)M3_STAGE1_DRIVER_TEETH) *
      ((float)M3_STAGE2_DRIVEN_TEETH / (float)M3_STAGE2_DRIVER_TEETH);

  return (M3_STEPPER_STEPS_PER_REV * motorRevPerScrewRev) /
         M3_SCREW_LEAD_MM_PER_REV;
}

inline long m3DistanceMmToSteps(float distanceMm) {
  return lroundf(distanceMm * m3ConfiguredStepsPerMm());
}

// --- Posiciones físicas de M3 configuradas en milímetros ---
extern const float M3_MIN_MM;
extern const float M3_HOME_MM;
extern const float M3_MAX_MM;

// Valores derivados usados internamente por el control de motores.
extern const long M3_MIN_STEPS_PHYS;
extern const long M3_MAX_STEPS_PHYS;
extern const long M3_HOME_STEPS_PHYS;

// --- Homing ---
extern const long HOMING_FAST_CHUNK;
extern const long HOMING_SLOW_CHUNK;
extern const long HOMING_BACKOFF_STEPS;
extern const uint32_t HOMING_STEP_PAUSE_MS;
extern const long MANUAL_M3_MAX_DELTA_STEPS;
extern const uint16_t MANUAL_M3_PULSE_US;
extern const uint16_t CARTESIAN_SYNC_PULSE_US;
extern const uint16_t M3_DESCENT_SYNC_PULSE_US;
extern const uint16_t M3_ASCENT_SYNC_PULSE_US;
extern const uint16_t AUTO_M3_DESCENT_PULSE_US;
extern const uint16_t AUTO_M3_ASCENT_PULSE_US;
extern const bool M3_REZERO_ON_ENDSTOP;

// --- GUI / secuencias ---
extern const float DEFAULT_SAFE_Z_MM;
extern const float DEFAULT_PICK_Z_MM;
extern const float DEFAULT_PLACE_Z_MM;

// ============================================================================
// VISIÓN / BANDA / PREDICCIÓN
// ============================================================================

// Por seguridad permanecerá false hasta terminar
// la calibración física.
extern const bool AUTO_PICK_ENABLED;


// -----------------------------------------------------
// Encoder
// -----------------------------------------------------

// Actualmente false porque todavía no hay encoder.
extern const bool BELT_ENCODER_ENABLED;
extern const uint8_t BELT_ENCODER_CHANNEL;
extern const int32_t ENCODER_PULSES_PER_REV;
extern const int32_t ENCODER_QUADRATURE_MULTIPLIER;
extern const float ENCODER_ROLLER_DIAMETER_MM;
extern const int8_t ENCODER_DIRECTION_SIGN;

// mm recorridos por cada cuenta del encoder.
// PENDIENTE medir.
extern const float ENCODER_MM_PER_COUNT;


// -----------------------------------------------------
// Cámara -> coordenadas de banda
// -----------------------------------------------------

// Si false:
// cameraX -> longitudinal
// cameraY -> transversal
//
// Si true:
// cameraY -> longitudinal
// cameraX -> transversal
extern const bool CAMERA_SWAP_AXES;

// Permiten invertir los ejes sin cambiar la homografia.
extern const float CAMERA_LONGITUDINAL_SIGN;
extern const float CAMERA_TRANSVERSE_SIGN;


// -----------------------------------------------------
// Posición física cámara -> zona de robot
// -----------------------------------------------------

// Distancia D.
// PENDIENTE medir.
//
// Se mide desde la referencia longitudinal 0 de
// la cámara hasta el origen longitudinal de la
// zona de pick.
extern const float CAMERA_TO_PICK_D_MM;


// Coordenada transversal que consideraremos
// centro de la banda.
extern const float BELT_TRANSVERSE_ZERO_MM;


// -----------------------------------------------------
// Relación banda -> sistema cartesiano SCARA
// -----------------------------------------------------

// Coordenadas XY del origen de la zona de pick
// expresadas en el sistema del robot.
extern const float BELT_PICK_ORIGIN_ROBOT_X_MM;
extern const float BELT_PICK_ORIGIN_ROBOT_Y_MM;

// Ángulo del eje longitudinal de la banda
// respecto a +X del robot.
extern const float BELT_ANGLE_RAD;


// -----------------------------------------------------
// Zona permitida de intercepción
// -----------------------------------------------------

extern const float PICK_AREA_X_MIN_MM;
extern const float PICK_AREA_X_MAX_MM;

extern const float PICK_AREA_Y_MIN_MM;
extern const float PICK_AREA_Y_MAX_MM;


inline PinStatus dirPosFromNeg(PinStatus negLevel) {
  return (negLevel == LOW) ? HIGH : LOW;
}

extern const bool BELT_GEOMETRY_CALIBRATED;

// Transformacion cartesiana obtenida con puntos del modo TEACH.
extern const bool TEACH_MAP_ENABLED;
extern const float TEACH_CAM_X_MIN_MM;
extern const float TEACH_CAM_X_MAX_MM;
extern const float TEACH_CAM_Y_MIN_MM;
extern const float TEACH_CAM_Y_MAX_MM;
extern const float TEACH_ROBOT_X_FROM_CAM_X;
extern const float TEACH_ROBOT_X_FROM_CAM_Y;
extern const float TEACH_ROBOT_X_OFFSET;
extern const float TEACH_ROBOT_Y_FROM_CAM_X;
extern const float TEACH_ROBOT_Y_FROM_CAM_Y;
extern const float TEACH_ROBOT_Y_OFFSET;

// Primera etapa de PICK por encoder.
extern const bool AUTO_PICK_DESCENT_ENABLED;
extern const float AUTO_PICK_MIN_SPEED_MM_S;
extern const float AUTO_PICK_MISSED_DISTANCE_MM;
extern const float AUTO_PICK_Z_MM;
extern const float AUTO_PICK_HOVER_ABOVE_MM;
extern const float AUTO_FINAL_DESCENT_ESTIMATE_S;
extern const float AUTO_FINAL_DESCENT_HORIZONTAL_ESTIMATE_S;
extern const float AUTO_PICK_CLOSE_LEAD_MM;
extern const float AUTO_PICK_CLOSE_MAX_ABOVE_HORIZONTAL_MM;
extern const float AUTO_PICK_CLOSE_LEAD_VERTICAL_MM;
extern const float AUTO_PICK_CAPTURE_MAX_LATE_MM;
extern const float AUTO_PICK_X_CORRECTION_MM;
extern const float AUTO_PICK_Y_CORRECTION_MM;
extern const bool AUTO_PICK_TIMING_FOLLOWS_XY_CORRECTION;
extern const float AUTO_PICK_RETRACT_Z_MM;
extern const uint8_t GRIPPER_OPEN_ANGLE;
extern const uint8_t GRIPPER_CLOSED_ANGLE;
extern const uint8_t AUTO_ROTATION_VERTICAL_ANGLE;
extern const uint8_t AUTO_ROTATION_HORIZONTAL_ANGLE;
extern const bool AUTO_ROTATION_COMPENSATION_ENABLED;
extern const float AUTO_ROTATION_S0_DEG;
extern const float AUTO_ROTATION_ZERO_X_OFFSET_DEG;
extern const float AUTO_ROTATION_GAIN;
extern const float AUTO_ROTATION_CAMERA_Y_ROBOT_DEG;
extern const uint8_t AUTO_ROTATION_MIN_ANGLE;
extern const uint8_t AUTO_ROTATION_MAX_ANGLE;
extern const uint32_t GRIPPER_SETTLE_MS;
extern const uint32_t GRIPPER_CLOSE_SETTLE_MS;
extern const float PLACE_TRAVEL_Z_MM;
extern const float PLACE_PIECE1_X_MM;
extern const float PLACE_PIECE1_Y_MM;
extern const float PLACE_PIECE1_Z_MM;
extern const float PLACE_PIECE2_X_MM;
extern const float PLACE_PIECE2_Y_MM;
extern const float PLACE_PIECE2_Z_MM;
extern const float PLACE_PIECE5_X_MM;
extern const float PLACE_PIECE5_Y_MM;
extern const float PLACE_PIECE5_Z_MM;
extern const uint32_t AUTO_TRACK_NO_PROGRESS_TIMEOUT_MS;
extern const int32_t AUTO_TRACK_PROGRESS_MIN_COUNTS;

// ======================================================
// MOCK CAMARA -> ROBOT
// ======================================================

// true = usar posición de cámara para generar
// un objetivo provisional dentro del workspace del SCARA.
extern const bool MOCK_VISION_ENABLED;

// false = solamente calcula/imprime.
// true  = permite movimiento físico.
extern const bool MOCK_MOVE_ENABLED;

// Cuánto puede alejarse el mock del HOME en steps.
// Primera prueba: +/-500 steps.
extern const long MOCK_M1_HALF_RANGE_STEPS;
extern const long MOCK_M2_HALF_RANGE_STEPS;
