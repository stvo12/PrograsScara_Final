#pragma once

#include <Arduino.h>

struct PadState {
  uint8_t  menuState = 0;   // 0..4  (1=Cal,2=Manual,3=Auto,4=Probar DI)
  uint8_t  manualMode = 0;  // 0=none, 1=articular, 2=cartesiano
  uint8_t  calibrationMode = 0; // 1=completa, 2=solo brazo, 3=solo camara, 4=solo M3
  uint8_t  misc = 0;
  bool     estop = false;

  uint8_t  selectedSource = 0; // 0=LOCAL, 1=PC, 2=AUTO
  bool     pmcStatusValid = false;
  uint8_t  pmcRunState = 0;
  uint8_t  pmcFlags = 0;
  uint8_t  pmcActiveSource = 0;
  // Datos de vision recibidos desde la ESP32 por comando 'N'
  // huskyFlags:
  // bit0 = Husky inicializada
  // bit1 = modelo seleccionado
  // bit2 = pieza detectada
  // bit3 = ultima lectura OK
  // bit4 = error
  // bit5 = conversion pixel -> mm valida
  // bit6 = nombre de pieza valido
  uint8_t  huskyFlags = 0;
  bool     huskyDetected = false;
  char     huskyName[17] = {0};
  uint8_t  huskyCount = 0;
  int16_t  huskyX = 0;
  int16_t  huskyY = 0;
  int16_t  huskyW = 0;
  int16_t  huskyH = 0;

  // Coordenadas fisicas de la pieza dentro
  // del rectangulo de calibracion.
  // Estan expresadas en decimas de mm.
  int16_t huskyXmm10 = 0;
  int16_t huskyYmm10 = 0;

  bool huskyMapped = false;

  uint16_t buttons = 0;
  uint8_t  dpad = 0;
  int8_t   lx = 0, ly = 0, rx = 0, ry = 0;
  uint8_t  lt = 0, rt = 0;
};

enum CalibrationMode : uint8_t {
  CALIBRATION_MODE_NONE = 0,
  CALIBRATION_MODE_FULL = 1,
  CALIBRATION_MODE_ARM = 2,
  CALIBRATION_MODE_CAMERA = 3,
  CALIBRATION_MODE_M3 = 4
};

enum ManualMode : uint8_t {
  MANUAL_MODE_NONE = 0,
  MANUAL_MODE_JOINT = 1,
  MANUAL_MODE_CARTESIAN = 2
};

enum ControlSource : uint8_t {
  CONTROL_SOURCE_NONE = 0,
  CONTROL_SOURCE_ESP,
  CONTROL_SOURCE_GUI,
  CONTROL_SOURCE_AUTO
};

enum PanelControlSource : uint8_t {
  PANEL_SOURCE_LOCAL = 0,
  PANEL_SOURCE_PC = 1,
  PANEL_SOURCE_AUTO = 2
};

enum RobotMode : uint8_t {
  ROBOT_MODE_IDLE = 0,
  ROBOT_MODE_CALIBRATION = 1,
  ROBOT_MODE_MANUAL = 2,
  ROBOT_MODE_AUTO = 3,
  ROBOT_MODE_TEST_DI = 4,
  ROBOT_MODE_TEACH = 5
};

enum PmcRunState : uint8_t {
  PMC_RUN_UNKNOWN = 0,
  PMC_RUN_READY = 1,
  PMC_RUN_BUSY = 2,
  PMC_RUN_HOMING = 3,
  PMC_RUN_AUTO_RUNNING = 4,
  PMC_RUN_ERROR = 5
};

enum GuiCommandType : uint8_t {
  GUI_CMD_NONE = 0,
  GUI_CMD_GET_STATUS,
  GUI_CMD_HOME,
  GUI_CMD_HOME_M3,
  GUI_CMD_ENABLE,
  GUI_CMD_DISABLE,
  GUI_CMD_ESTOP,
  GUI_CMD_RESET_ESTOP,
  GUI_CMD_GOTO_XYZ,
  GUI_CMD_MOVE_JOINTS,
  GUI_CMD_RUN_PICK,
  GUI_CMD_RUN_PICKPLACE,
};

struct GuiCommand {
  GuiCommandType type = GUI_CMD_NONE;
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  float d = 0.0f;
  bool pending = false;
};

struct PickPlaceParams {
  float pickX = 0.0f;
  float pickY = 0.0f;
  float placeX = 0.0f;
  float placeY = 0.0f;
  float safeZ = 0.0f;
  float pickZ = 0.0f;
  float placeZ = 0.0f;
  uint8_t graspAngle = 0;
};
