#include <Wire.h>
#include <cstdio>

#include "ScaraCalibrationCore.h"
#include "project_config.h"
#include "types.h"
#include "robot_state.h"
#include "esp_interface.h"
#include "motion_service.h"
#include "calibration_service.h"
#include "command_dispatcher.h"
#include "status_report.h"
#include "gui_protocol.h"
#include "sequence_runner.h"
#include "kinematics.h"
#include "scara_pose.h"
#include "motor_map.h"
#include "safety.h"
#include "belt_tracking.h"

// ======================================================
// CONFIGURACIÓN EDITABLE
// ======================================================

// --- Comunicación ---
extern const int SERIAL_BAUDRATE = 115200;
extern const uint8_t ESP32_ADDR = 0x28;
extern const uint8_t HUSKY_ESP32_ADDR = 0x29;
extern const uint32_t I2C_FREQ = 100000;
// Tres muestras consecutivas eliminan pulsos espurios del bit ESTOP.
extern const uint8_t ESTOP_CONFIRM_SAMPLES = 3;
extern const uint32_t PRINT_MS = 120;
extern const uint32_t LOOP_DELAY_MS = 5;

// --- Canales de salida PMC ---
extern const uint8_t STEP1_CH_CFG = 0;
extern const uint8_t DIR1_CH_CFG = 1;
extern const uint8_t STEP2_CH_CFG = 2;
extern const uint8_t DIR2_CH_CFG = 3;
extern const uint8_t STEP3_CH_CFG = 4;
extern const uint8_t DIR3_CH_CFG = 5;
extern const uint8_t ENA_CH_CFG = 6;

// --- Lógica de drivers ---
extern const PinStatus ENA_LEVEL_ENABLE_CFG = LOW;
extern const PinStatus ENA_LEVEL_DISABLE_CFG = HIGH;
extern const PinStatus DIR1_LEVEL_NEG_CFG = LOW;
extern const PinStatus DIR2_LEVEL_NEG_CFG = LOW;
extern const PinStatus DIR3_LEVEL_NEG_CFG = LOW;

// --- Endstops / entradas digitales ---
extern const int ENDSTOP_ACTIVE_LEVEL = 0;  // 0 = LOW, 1 = HIGH
extern const int DI_TOTAL = 7;
extern const int DI_PINS[] = {
  DIN_READ_CH_PIN_00,
  DIN_READ_CH_PIN_01,
  DIN_READ_CH_PIN_02,
  DIN_READ_CH_PIN_03,
  DIN_READ_CH_PIN_04,
  DIN_READ_CH_PIN_05,
  DIN_READ_CH_PIN_06
};

// --- Geometria y transmision articular ---
// Longitudes entre centros de ejes, no longitud exterior de las piezas.
extern const float SCARA_L1_MM = 120.5f;
extern const float SCARA_L2_MM = 120.0f;
extern const uint16_t M2_MOTOR_PULLEY_TEETH = 42;
extern const uint16_t M2_JOINT_PULLEY_TEETH = 116;

// --- Transmision Z configurable ---
// Una etapa directa: stepper 44T (motriz) -> tornillo Z 39T.
extern const float M3_STEPPER_STEPS_PER_REV = 1600.0f;
extern const uint16_t M3_STAGE1_DRIVER_TEETH = 50;
extern const uint16_t M3_STAGE1_DRIVEN_TEETH = 39;
// Sin segunda etapa: relacion neutra 1:1 para conservar la conversion existente.
extern const uint16_t M3_STAGE2_DRIVER_TEETH = 1;
extern const uint16_t M3_STAGE2_DRIVEN_TEETH = 1;
// Avance lineal real por revolucion: paso 2 mm x 4 entradas = 8 mm.
// extern const float M3_SCREW_LEAD_MM_PER_REV = 2.0f;
extern const float M3_SCREW_LEAD_MM_PER_REV = 8.0f;

// --- Posiciones fisicas M3 en milimetros desde el final de carrera/pick ---
extern const float M3_MIN_MM = 0.0f;
extern const float M3_HOME_MM = 115.0f;
extern const float M3_MAX_MM = 130.0f;

// Valores anteriores introducidos manualmente en pasos:
// extern const long M3_MAX_STEPS_PHYS = 70400;
// extern const long M3_HOME_STEPS_PHYS = 45000;
// extern const long M3_HOME_STEPS_PHYS = 30000;
// extern const long M3_HOME_STEPS_PHYS = 1000;

// Conversión automática usando dientes, micropasos y avance del tornillo.
extern const long M3_MIN_STEPS_PHYS = m3DistanceMmToSteps(M3_MIN_MM);
extern const long M3_HOME_STEPS_PHYS = m3DistanceMmToSteps(M3_HOME_MM);
extern const long M3_MAX_STEPS_PHYS = m3DistanceMmToSteps(M3_MAX_MM);

// --- Homing ---
extern const long HOMING_FAST_CHUNK = 40;
extern const long HOMING_SLOW_CHUNK = 8;
extern const long HOMING_BACKOFF_STEPS = 200;
extern const uint32_t HOMING_STEP_PAUSE_MS = 1;

// Jog manual M3: ligeramente mas rapido que homing (100 us).
// Solo se aplica cuando M1/M2 están quietos.
extern const long MANUAL_M3_MAX_DELTA_STEPS = 48;
extern const uint16_t MANUAL_M3_PULSE_US = 80;

// Semiperiodo STEP para movimientos cartesianos (pick/place/GOTO).
// Menor valor = mayor velocidad. No modifica homing ni calibracion.
// 55 us acelera ligeramente XY cuando M3 no se mueve. Verificar fisicamente
// que no pierda pasos; con M3 activo se usa su velocidad coordinada separada.
// extern const uint16_t CARTESIAN_SYNC_PULSE_US = 200;
extern const uint16_t CARTESIAN_SYNC_PULSE_US = 55;

// Velocidades separadas para M3 durante movimientos cartesianos.
// Descenso rapido para interceptar la pieza; ascenso lento para conservar
// torque y evitar perdida de pasos al levantar la carga.
extern const uint16_t M3_DESCENT_SYNC_PULSE_US = 62;
extern const uint16_t M3_ASCENT_SYNC_PULSE_US = 200;

// M3 en Auto: semiperiodos STEP independientes de GOTO/manual/calibracion.
// Restaurar el ritmo previo a la separacion de velocidades de Auto.
extern const uint16_t AUTO_M3_DESCENT_PULSE_US = 55;
extern const uint16_t AUTO_M3_ASCENT_PULSE_US = 200;
// Global (manual, comandos, Auto y calibracion): switch minimo confirmado al
// bajar fija el cero sin modificar el rango ni los pasos nominales de HOME.
extern const bool M3_REZERO_ON_ENDSTOP = true;

// --- Defaults para GUI / secuencias ---
extern const float DEFAULT_SAFE_Z_MM = 60.0f;
extern const float DEFAULT_PICK_Z_MM = 20.0f;
extern const float DEFAULT_PLACE_Z_MM = 20.0f;

// ======================================================
// VISIÓN / BANDA / PREDICCIÓN
// ======================================================

// BLOQUEO GENERAL.
// NO cambiar a true hasta tener calibración física.
extern const bool AUTO_PICK_ENABLED = true;


// ------------------------------------------------------
// Encoder
// ------------------------------------------------------

// Encoder Omron E6B2-CWZ6C conectado en A0/B0/Z0.
// La biblioteca PMC cuenta en cuadratura X2 por defecto.
extern const bool BELT_ENCODER_ENABLED = true;
extern const uint8_t BELT_ENCODER_CHANNEL = 0;
extern const int32_t ENCODER_PULSES_PER_REV = 1024;
extern const int32_t ENCODER_QUADRATURE_MULTIPLIER = 2;
extern const float ENCODER_ROLLER_DIAMETER_MM = 50.0f;

// Cambiar a -1 si al avanzar la banda el conteo disminuye.
extern const int8_t ENCODER_DIRECTION_SIGN = -1;

extern const float ENCODER_MM_PER_COUNT =
    (PI * ENCODER_ROLLER_DIAMETER_MM) /
    (ENCODER_PULSES_PER_REV * ENCODER_QUADRATURE_MULTIPLIER);

// ------------------------------------------------------
// Orientación cámara -> banda
// ------------------------------------------------------

// La homografia entrega X transversal (Tag 1 -> Tag 2)
// y Y longitudinal (Tag 1 -> Tag 3), por eso se intercambian.
extern const bool CAMERA_SWAP_AXES = true;

extern const float CAMERA_LONGITUDINAL_SIGN = 1.0f;
extern const float CAMERA_TRANSVERSE_SIGN = 1.0f;


// ------------------------------------------------------
// Distancia cámara -> robot
// ------------------------------------------------------

// Distancia longitudinal desde la fila Tag 1-Tag 2 hasta la linea de pick.
// Mantener 540 solo si esa distancia se confirma fisicamente.
extern const float CAMERA_TO_PICK_D_MM = 650.0f;


// Centro transversal entre Tag 1 y Tag 2: 424 / 2.
extern const float BELT_TRANSVERSE_ZERO_MM = 212.0f;


// ------------------------------------------------------
// Banda -> robot
// ------------------------------------------------------

// Origen robot de la linea de pick en el centro transversal.
// CAD 2026-10-19: M1=(243.5,-289) y Tag1=(-446.5,212) en planta X/Z.
// X_R=501-X_C; Y_R=Y_C-690. Con D=650: origen=(289,-40).
extern const float BELT_PICK_ORIGIN_ROBOT_X_MM = 289.0f;
extern const float BELT_PICK_ORIGIN_ROBOT_Y_MM =
    CAMERA_TO_PICK_D_MM - 690.0f;

// El avance longitudinal de la banda apunta a +Y del robot.
extern const float BELT_ANGLE_RAD = 1.57079632679f; //pi/2


// ------------------------------------------------------
// Workspace permitido para Auto
// ------------------------------------------------------

// Valores provisionales.
// NO se usarán para mover porque AUTO_PICK_ENABLED=false.
extern const float PICK_AREA_X_MIN_MM = -1000.0f;
extern const float PICK_AREA_X_MAX_MM =  1000.0f;

extern const float PICK_AREA_Y_MIN_MM = -1000.0f;
extern const float PICK_AREA_Y_MAX_MM =  1000.0f;


// En AUTO, M3 espera arriba y comienza el descenso antes de la linea de
// pick para que la garra cierre ligeramente antes de cruzarla.
// GRIPPER_CLOSE_SETTLE_MS conserva la espera posterior al cierre.

extern const bool FORCE_GUI_MODE_NO_ESP = true;

extern const bool BELT_GEOMETRY_CALIBRATED = true;

// Transformacion afín cámara -> linea de pick, calculada con 6 puntos TEACH.
// Fuera de la region enseñada el objetivo se rechaza por seguridad.
extern const bool TEACH_MAP_ENABLED = false;
extern const float TEACH_CAM_X_MIN_MM = 96.2f;
extern const float TEACH_CAM_X_MAX_MM = 424.0f;
extern const float TEACH_CAM_Y_MIN_MM = 0.0f;
extern const float TEACH_CAM_Y_MAX_MM = 248.0f;
extern const float TEACH_ROBOT_X_FROM_CAM_X = -1.1713269f;
extern const float TEACH_ROBOT_X_FROM_CAM_Y = -0.1377568f;
extern const float TEACH_ROBOT_X_OFFSET = 506.15234f;
extern const float TEACH_ROBOT_Y_FROM_CAM_X = -1.0934368f;
extern const float TEACH_ROBOT_Y_FROM_CAM_Y = -0.1338832f;
extern const float TEACH_ROBOT_Y_OFFSET = 70.95670f;

// PICK etapa 1: preposicion y sincronizacion habilitadas.
// El descenso permanece bloqueado hasta medir Z y definir la pinza.
extern const bool AUTO_PICK_DESCENT_ENABLED = true;
extern const float AUTO_PICK_MIN_SPEED_MM_S = 2.0f;
extern const float AUTO_PICK_MISSED_DISTANCE_MM = 80.0f;
// DI06 define Z=0; por deflexion la captura se hace 5 mm arriba.
// Espera absoluta Z=20 mm: 15 mm por encima del punto de captura.
extern const float AUTO_PICK_Z_MM = 0.0f;
extern const float AUTO_PICK_HOVER_ABOVE_MM = 20.0f;
// Iniciar el descenso con margen para que termine antes del cierre anticipado.
// El tiempo real se imprime como AUTO_FINAL_DESCENT_DONE.
extern const float AUTO_FINAL_DESCENT_ESTIMATE_S = 1.30;
// Horizontal: aproximacion desde arriba mas tarde, con cierre durante descenso.
// Adelanto de inicio horizontal: puede ser menor que la duracion del descenso
// porque el cierre se coordina durante el movimiento, no espera a terminarlo.
// Valor de prueba para dejar pasar la pieza antes de que los dedos bajen.
extern const float AUTO_FINAL_DESCENT_HORIZONTAL_ESTIMATE_S = 0.80f;
// Horizontal: ordenar cierre durante el descenso final segun el encoder.
// Vertical: terminar descenso y cerrar antes de la linea de pick.
// Ajustar con pruebas fisicas del servo y de la zona efectiva de agarre.
extern const float AUTO_PICK_CLOSE_LEAD_MM = 35.0f;
// Horizontal: no ordenar cierre hasta estar a esta altura sobre el pick.
extern const float AUTO_PICK_CLOSE_MAX_ABOVE_HORIZONTAL_MM = 10.0f;
extern const float AUTO_PICK_CLOSE_LEAD_VERTICAL_MM = 55.0f;
extern const float AUTO_PICK_CAPTURE_MAX_LATE_MM = 10.0f;
// Ajustes empiricos del centro real de la pinza respecto al objetivo Auto.
// No alteran la linea de pick ni el instante de cierre por encoder.
extern const float AUTO_PICK_X_CORRECTION_MM = -25.0f;
extern const float AUTO_PICK_Y_CORRECTION_MM = -15.0f;
// true: el offset XY desplaza el punto fisico de captura y tambien su referencia
// temporal. false: offset solo compensa TCP; la linea temporal queda en D nominal.
extern const bool AUTO_PICK_TIMING_FOLLOWS_XY_CORRECTION = false;
extern const float AUTO_PICK_RETRACT_Z_MM = 60.0f;
extern const uint8_t GRIPPER_OPEN_ANGLE = 70;
extern const uint8_t GRIPPER_CLOSED_ANGLE = 150;
// Angulos del servo de giro para la orientacion detectada por Husky.
extern const uint8_t AUTO_ROTATION_VERTICAL_ANGLE = 40;
extern const uint8_t AUTO_ROTATION_HORIZONTAL_ANGLE = 100;
// Compensacion de orientacion por q1+q2 con cero medido sobre el eje Y.
extern const bool AUTO_ROTATION_COMPENSATION_ENABLED = true;
// A 92 grados, Y de la garra queda paralelo al ultimo eslabon.
extern const float AUTO_ROTATION_S0_DEG = 92.0f;
// En ese cero, X esta a -90 grados del eslabon (modulo 180).
extern const float AUTO_ROTATION_ZERO_X_OFFSET_DEG = -90.0f;
// Acople directo 1:1; giro horario visto desde arriba al aumentar el comando.
extern const float AUTO_ROTATION_GAIN = -1.0f;
// Direccion de Y de camara en el plano XY del robot (+Y = 90 grados).
extern const float AUTO_ROTATION_CAMERA_Y_ROBOT_DEG = 90.0f;
extern const uint8_t AUTO_ROTATION_MIN_ANGLE = 0;
extern const uint8_t AUTO_ROTATION_MAX_ANGLE = 180;
extern const uint32_t GRIPPER_SETTLE_MS = 500;
// Tiempo exclusivo para que la pinza termine de cerrar antes de elevar M3.
extern const uint32_t GRIPPER_CLOSE_SETTLE_MS = 500;

// Puntos de entrega obtenidos con el modo TEACH y la pieza sostenida.
// Traslado 5 mm mas alto; coincide con el limite configurado M3_MAX_MM.
extern const float PLACE_TRAVEL_Z_MM = 130.0f;
extern const float PLACE_PIECE1_X_MM = -36.522f;
extern const float PLACE_PIECE1_Y_MM = -76.514f;
extern const float PLACE_PIECE1_Z_MM = 125.000f;
extern const float PLACE_PIECE2_X_MM = -66.555f;
extern const float PLACE_PIECE2_Y_MM = -48.546f;
extern const float PLACE_PIECE2_Z_MM = 125.000f;
// Pieza5 ocupa la antigua descarga de Pieza3; coordenadas conservadas.
extern const float PLACE_PIECE5_X_MM = -105.081f;
extern const float PLACE_PIECE5_Y_MM = -12.370f;
extern const float PLACE_PIECE5_Z_MM = 125.000f;

// El tracker no vence por edad mientras exista progreso real del encoder.
// Se libera si el conteo deja de cambiar lo suficiente durante 2.5 segundos.
extern const uint32_t AUTO_TRACK_NO_PROGRESS_TIMEOUT_MS = 2500;
extern const int32_t AUTO_TRACK_PROGRESS_MIN_COUNTS = 20;

// ======================================================
// MOCK CAMARA -> ROBOT
// ======================================================

// Usa solamente coordenadas de cámara.
// No usa encoder ni distancia D.
extern const bool MOCK_VISION_ENABLED = false;

// Primera prueba:
// false = calcula e imprime
// true  = mueve físicamente
extern const bool MOCK_MOVE_ENABLED = false;

// Zona provisional alrededor del HOME.
// Con tu calibración:
//
// M1 HOME = 2940
// M2 HOME = 1816
//
// Por ahora permitimos +/-500 steps.
extern const long MOCK_M1_HALF_RANGE_STEPS = 500;
extern const long MOCK_M2_HALF_RANGE_STEPS = 500;

// ======================================================
// SETUP / LOOP
// ======================================================

void setup() {
  delay(1500);
  Serial.begin(SERIAL_BAUDRATE);
  delay(600);

  Serial.print("M3 steps/mm calculados = ");
  Serial.println(m3ConfiguredStepsPerMm(), 6);
  Serial.print("M3 HOME ");
  Serial.print(M3_HOME_MM, 1);
  Serial.print(" mm = ");
  Serial.print(M3_HOME_STEPS_PHYS);
  Serial.println(" steps");
  Serial.print("M3 MAX ");
  Serial.print(M3_MAX_MM, 1);
  Serial.print(" mm = ");
  Serial.print(M3_MAX_STEPS_PHYS);
  Serial.println(" steps");

  espInterfaceInit();
  motionServiceInitHardware();
  MachineControl_DigitalInputs.begin();
  guiProtocolInit();
  sequenceRunnerInit();
  beltTrackingInit();
}

void loop() {
  updateBeltEstimator();
  updateGuiProtocol();
  dispatchPendingGuiCommands();

  static uint8_t i2cFailCount = 0;
  static uint8_t estopActiveSamples = 0;
  static PadState lastValidState{};

  PadState st{};
  bool ok = readPadStateI2C(st);

  if (ok) {
    i2cFailCount = 0;
    if (st.estop) {
      if (estopActiveSamples < ESTOP_CONFIRM_SAMPLES) estopActiveSamples++;
      if (estopActiveSamples >= ESTOP_CONFIRM_SAMPLES && !g_remoteEstop) {
        g_remoteEstop = true;
        activateEmergencyStop();
      }
    } else {
      estopActiveSamples = 0;
      if (g_remoteEstop) {
        g_remoteEstop = false;
        releaseEmergencyStop();
      }
    }
    // El resto del programa solo recibe el ESTOP ya confirmado, no la
    // muestra cruda que podria contener un pulso aislado.
    st.estop = g_remoteEstop;
    lastValidState = st;
  } else {
    i2cFailCount++;
    estopActiveSamples = 0;

    // Durante los primeros fallos, no tirar error todavía.
    // Usamos el último estado válido para no mandar todo a STOP de inmediato.
    st = lastValidState;
  }

  printPadStateIfNeeded(st, ok);

  // Si falló varias veces seguidas, ahora sí declarar desconexión real
  if (!ok && i2cFailCount >= 5) {
    resetRobotStateForStop();
    setRobotError("ESP_I2C_OFFLINE");
    // Fallo de comunicacion: STOP normal. Se conserva la referencia y el torque.
    publishPmcStatusToEspIfNeeded(true);
    delay(LOOP_DELAY_MS);
    return;
  }

  // Si falló solo 1, 2, 3 o 4 veces, seguimos sin tumbar todo.
  // Pero evitamos ejecutar control si no hubo lectura nueva.
  if (!ok) {
    publishPmcStatusToEspIfNeeded();
    delay(LOOP_DELAY_MS);
    return;
  }

  dispatchPadState(st);
  updateSequenceRunner();
  publishPmcStatusToEspIfNeeded();

  delay(LOOP_DELAY_MS);
}
