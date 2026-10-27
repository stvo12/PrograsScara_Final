#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ESP32-WROOM-32 (reemplazo del WROVER-E)
// I2C esclavo PMC: SDA 27, SCL 14, direccion 0x28
// I2C maestro OLED: SDA 21, SCL 22, RESET 26, direccion 0x3C
// PWM: garra 1 = 25, garra 2 = 33, giro = 32
// IMPORTANTE: usar tierra comun y alimentar los servos con una fuente de 5 V
// independiente, capaz de entregar la corriente de los tres servos.

// =====================================================
// CONFIGURACION EDITABLE
// =====================================================

// ===== SERVOS DE LA GARRA Y GIRO =====
static const int SERVO_GRIPPER_1_PIN = 25;
static const int SERVO_GRIPPER_2_PIN = 33;
static const int SERVO_ROTATION_PIN = 32;
static const int GRIPPER_OPEN_ANGLE = 74;
static const int GRIPPER_CLOSED_ANGLE = 148;
static const int ROTATION_MIN_ANGLE = 0;
static const int ROTATION_MAX_ANGLE = 180;
static const int ROTATION_INITIAL_ANGLE = 40;
// true si el segundo servo requiere girar en sentido opuesto.
static const bool SERVO_GRIPPER_2_INVERTED = false;

static int clampGripperAngle(int angle) {
  if (angle < GRIPPER_OPEN_ANGLE) return GRIPPER_OPEN_ANGLE;
  if (angle > GRIPPER_CLOSED_ANGLE) return GRIPPER_CLOSED_ANGLE;
  return angle;
}

static int clampRotationAngle(int angle) {
  if (angle < ROTATION_MIN_ANGLE) return ROTATION_MIN_ANGLE;
  if (angle > ROTATION_MAX_ANGLE) return ROTATION_MAX_ANGLE;
  return angle;
}

int servo180Angle = GRIPPER_OPEN_ANGLE;
int servoRotationAngle = ROTATION_INITIAL_ANGLE;

// ===== I2C ESCLAVO HACIA PORTENTA =====
static const uint8_t I2C_ADDR = 0x28;
static const int SDA_PIN = 27;
static const int SCL_PIN = 14;
static const uint32_t I2C_FREQ = 100000;

// ===== OLED =====
static const int OLED_SDA = 21;
static const int OLED_SCL = 22;
static const int OLED_RESET = 26;
static const uint8_t OLED_ADDR = 0x3C;
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;

// ===== GENERALES =====
static const uint32_t PRINT_PERIOD_MS = 150;
static const uint32_t MENU_DEBOUNCE_MS = 180;
static const uint8_t MENU_ITEMS_PER_PAGE = 3;

// ===== BOTONES / DPAD =====
static const uint8_t DPAD_MASK_UP = 0x01;
static const uint8_t DPAD_MASK_DOWN = 0x02;
static const uint16_t BTN_SELECT = 0x0001;  // X en tu mapeo actual
static const uint16_t BTN_CIRCLE = 0x0002;  // Circulo / B en Bluepad32
static const uint16_t BTN_SQUARE = 0x0004;  // Cuadrado / X en Bluepad32
static const uint16_t BTN_L1 = 0x0010;
static const uint16_t BTN_R1 = 0x0020;
static const uint16_t BTN_L2 = 0x0040;
static const uint16_t BTN_R2 = 0x0080;
static const uint8_t MISC_SHARE = 0x02;
static const uint8_t MISC_OPTION = 0x04;

// ===== FLAGS DESDE/HACIA PMC =====
static const uint8_t PMC_FLAG_ESTOP           = 0x01;
static const uint8_t PMC_FLAG_HOMED           = 0x02;
static const uint8_t PMC_FLAG_DRIVERS_ENABLED = 0x04;
static const uint8_t PMC_FLAG_BUSY            = 0x08;
static const uint8_t PMC_FLAG_ERROR           = 0x10;

// =====================================================
// OBJETOS GLOBALES
// =====================================================
Servo servo180;
Servo servoSecond;
Servo servoRotation;

void writeGripperServos(int angle) {
  angle = clampGripperAngle(angle);
  servo180.write(angle);
  servoSecond.write(SERVO_GRIPPER_2_INVERTED ? 180 - angle : angle);
}

void writeRotationServo(int angle) {
  servoRotation.write(clampRotationAngle(angle));
}

TwoWire WireOLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &WireOLED, OLED_RESET);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool squareWasPressed[BP32_MAX_GAMEPADS] = {false};

// ===== DATOS GAMEPAD =====
uint16_t Boton[16]  = {0};
uint8_t  Cruz[8]    = {0};
int8_t   JAxisLx[8] = {0};
int8_t   JAxisLy[8] = {0};
int8_t   JAxisRx[8] = {0};
int8_t   JAxisRy[8] = {0};
uint8_t  L2[8]      = {0};
uint8_t  R2[8]      = {0};
uint8_t  Misc[8]    = {0};

uint8_t lastCmd = 'M';
uint32_t lastPrintMs = 0;
uint32_t lastMenuMoveMs = 0;
uint32_t lastGripperAdjustMs = 0;
uint32_t lastRotationAdjustMs = 0;
volatile bool g_gripperCommandPending = false;
volatile bool g_rotationCommandPending = false;

// Proteccion simple
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// ESTADOS Y ENUMS
// =====================================================
enum MenuItem : uint8_t {
  MENU_CAL = 0,
  MENU_MANUAL = 1,
  MENU_AUTO = 2,
  MENU_DI = 3,
  MENU_STATUS = 4,
  MENU_TEACH = 5,
  MENU_COUNT = 6
};

const uint8_t MENU_PAGE_COUNT = (MENU_COUNT + MENU_ITEMS_PER_PAGE - 1) / MENU_ITEMS_PER_PAGE;

enum ControlMode : uint8_t {
  MODE_MENU = 0,
  MODE_MANUAL_SELECT = 1,
  MODE_MANUAL = 2,
  MODE_CAL = 3,
  MODE_AUTO = 4,
  MODE_DI = 5,
  MODE_STATUS = 6,
  MODE_CAL_SELECT = 7,
  MODE_TEACH = 8
};

enum CalibrationMode : uint8_t {
  CAL_MODE_NONE = 0,
  CAL_MODE_FULL = 1,
  CAL_MODE_ARM = 2,
  CAL_MODE_CAMERA = 3,
  CAL_MODE_M3 = 4
};

enum ManualMode : uint8_t {
  MANUAL_MODE_NONE = 0,
  MANUAL_MODE_JOINT = 1,
  MANUAL_MODE_CARTESIAN = 2
};

enum ControlSource : uint8_t {
  CONTROL_SOURCE_LOCAL = 0,
  CONTROL_SOURCE_PC = 1,
  CONTROL_SOURCE_AUTO = 2
};

enum RobotRunState : uint8_t {
  ROBOT_STATE_UNKNOWN = 0,
  ROBOT_STATE_READY = 1,
  ROBOT_STATE_BUSY = 2,
  ROBOT_STATE_HOMING = 3,
  ROBOT_STATE_AUTO_RUNNING = 4,
  ROBOT_STATE_ERROR = 5
};

volatile uint8_t g_mode = MODE_MENU;
volatile uint8_t g_manualMode = MANUAL_MODE_NONE;
volatile uint8_t g_manualIndex = 0;   // 0=articular, 1=cartesiano
volatile uint8_t g_calibrationMode = CAL_MODE_NONE;
volatile uint8_t g_calibrationIndex = 0;
volatile uint8_t g_selectedControlSource = CONTROL_SOURCE_LOCAL;
volatile bool g_estop = false;
volatile uint8_t menuIndex = 0;
volatile uint8_t menuSelected = 0;
volatile bool g_lastOptionPressed = false;

// Estado reportado por la PMC (opcional, si la PMC lo escribe con cmd 'S')
volatile bool g_pmcStatusValid = false;
volatile uint8_t g_pmcRunState = ROBOT_STATE_UNKNOWN;
volatile uint8_t g_pmcFlags = 0;
volatile uint8_t g_pmcActiveSource = CONTROL_SOURCE_LOCAL;

// =====================================================
// HELPERS
// =====================================================
int8_t mapAxisToInt8(int v) {
  if (v > 512) v = 512;
  if (v < -512) v = -512;
  return (int8_t)((v * 127) / 512);
}

uint8_t mapTrigToUint8(int v) {
  if (v < 0) v = 0;
  if (v > 1023) v = 1023;
  return (uint8_t)((v * 255) / 1023);
}

const char* sourceToShort(uint8_t src) {
  switch (src) {
    case CONTROL_SOURCE_LOCAL: return "LOC";
    case CONTROL_SOURCE_PC:    return "PC";
    case CONTROL_SOURCE_AUTO:  return "AUT";
    default:                   return "---";
  }
}

const char* sourceToLong(uint8_t src) {
  switch (src) {
    case CONTROL_SOURCE_LOCAL: return "Local";
    case CONTROL_SOURCE_PC:    return "PC";
    case CONTROL_SOURCE_AUTO:  return "Auto";
    default:                   return "Desconocido";
  }
}

const char* robotStateToText(uint8_t st) {
  switch (st) {
    case ROBOT_STATE_READY:        return "READY";
    case ROBOT_STATE_BUSY:         return "BUSY";
    case ROBOT_STATE_HOMING:       return "HOMING";
    case ROBOT_STATE_AUTO_RUNNING: return "AUTO";
    case ROBOT_STATE_ERROR:        return "ERROR";
    default:                       return "UNKNOWN";
  }
}

void drawHeader(const char* title, bool estopActive) {
  display.setCursor(0, 0);
  display.print(title);

  uint8_t selectedSrc;
  portENTER_CRITICAL(&mux);
  selectedSrc = g_selectedControlSource;
  portEXIT_CRITICAL(&mux);

  display.setCursor(68, 0);
  display.print("SRC:");
  display.print(sourceToShort(selectedSrc));

  if (estopActive) {
    display.setCursor(0, 56);
    display.print("E-STOP ACTIVO");
  }
}

void setModeAndResetEstop(uint8_t newMode) {
  portENTER_CRITICAL(&mux);
  g_mode = newMode;
  g_lastOptionPressed = false;
  portEXIT_CRITICAL(&mux);
}

void resetToMenu() {
  portENTER_CRITICAL(&mux);
  g_mode = MODE_MENU;
  menuSelected = 0;
  g_manualMode = MANUAL_MODE_NONE;
  g_manualIndex = 0;
  g_lastOptionPressed = false;
  portEXIT_CRITICAL(&mux);
}

void updateOptionToggle(uint8_t misc) {
  bool optPressed = (misc & MISC_OPTION) != 0;
  portENTER_CRITICAL(&mux);
  bool lastOpt = g_lastOptionPressed;
  if (optPressed && !lastOpt) {
    g_estop = !g_estop;
  }
  g_lastOptionPressed = optPressed;
  portEXIT_CRITICAL(&mux);
}

// =====================================================
// BLUEPAD32 CALLBACKS
// =====================================================
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      squareWasPressed[i] = false;
      Serial.printf("Controller conectado, index=%d\n", i);
      break;
    }
  }
  ctl->setColorLED(108, 70, 117);
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      squareWasPressed[i] = false;
      Serial.printf("Controller desconectado, index=%d\n", i);
      break;
    }
  }
}

// =====================================================
// OLED
// =====================================================
void drawMenu(uint8_t idx, uint8_t selectedFlag, bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  uint8_t page = idx / MENU_ITEMS_PER_PAGE;
  uint8_t start = page * MENU_ITEMS_PER_PAGE;
  uint8_t end = start + MENU_ITEMS_PER_PAGE;
  if (end > MENU_COUNT) end = MENU_COUNT;

  drawHeader("MENU", estopActive);

  display.setCursor(0, 8);
  display.printf("Pag %u/%u", page + 1, MENU_PAGE_COUNT);

  const char* items[MENU_COUNT] = {
    "Calibracion",
    "Modo manual",
    "Modo automatico",
    "Probar DI",
    "Estado",
    "Modo Teach"
  };

  for (uint8_t i = start; i < end; i++) {
    uint8_t row = i - start;
    int y = 20 + row * 12;
    display.setCursor(0, y);
    display.print((i == idx) ? ">" : " ");
    display.setCursor(10, y);
    display.print(items[i]);
    if (selectedFlag && (selectedFlag - 1) == i) {
      display.setCursor(118, y);
      display.print("*");
    }
  }

  display.display();
}

void drawManualSelectScreen(uint8_t manualIndex, bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawHeader("MANUAL", estopActive);

  display.setCursor(0, 16);
  display.print((manualIndex == 0) ? "> " : "  ");
  display.println("Articular");

  display.setCursor(0, 30);
  display.print((manualIndex == 1) ? "> " : "  ");
  display.println("Cartesiano");

  display.setCursor(0, 44);
  display.print((manualIndex == 2) ? "> " : "  ");
  display.println("GUI PC");

  display.display();
}

void drawCalibrationSelectScreen(uint8_t calibrationIndex, bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawHeader("CALIBRACION", estopActive);
  const char* items[4] = {"Completa", "Solo brazo", "Solo camara", "Solo M3"};
  for (uint8_t i = 0; i < 4; i++) {
    display.setCursor(0, 16 + i * 11);
    display.print((calibrationIndex == i) ? "> " : "  ");
    display.println(items[i]);
  }
  display.display();
}

void drawManualScreen(bool estopActive, uint8_t manualMode) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawHeader("MODO MANUAL", estopActive);

  display.setCursor(0, 16);
  display.print("Submodo: ");
  if (manualMode == MANUAL_MODE_JOINT) {
    display.println("ARTICULAR");
    display.println("LX/LY/RY -> J1/J2/J3");
  } else {
    display.println("CARTESIANO");
    display.println("LX/LY/RY -> X/Y/Z");
  }

  display.setCursor(0, 40);
  display.print("Pinza:");
  display.print(servo180Angle);
  display.println(" L1/R1");
  display.print("Giro:");
  display.print(servoRotationAngle);
  display.println(" L2/R2");
  display.println("Share:menu Opt:E-stop");
  display.display();
}

void drawCalibrationScreen(bool estopActive, bool done, uint8_t calibrationMode) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawHeader("CALIBRACION", estopActive);
  display.setCursor(0, 18);
  if (calibrationMode == CAL_MODE_FULL) display.println("Completa");
  else if (calibrationMode == CAL_MODE_ARM) display.println("Solo brazo");
  else if (calibrationMode == CAL_MODE_M3) display.println("Solo M3");
  else display.println("Solo camara");
  if (!done) {
    display.println("Calibrando...");
  } else {
    display.println("Calibracion");
    display.println("terminada");
  }

  display.setCursor(0, 42);
  display.println("Share: menu");
  display.println("Opt: E-STOP");
  display.display();
}

void drawAutoScreen(bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawHeader("MODO AUTO", estopActive);

  display.setCursor(0, 14);
  display.println("Vision externa");
  display.println("ESP Husky: 0x29");
  display.println("");
  display.println("La PMC lee la");
  display.println("camara aparte.");

  display.setCursor(0, 56);
  display.println("Share: menu");

  display.display();
}

void drawTeachScreen(bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawHeader("MODO TEACH", estopActive);
  display.setCursor(0, 16);
  display.println("Manual articular");
  display.println("LX/LY/RY: M1/M2/M3");
  display.println("Circulo: guardar");
  display.println("Share: menu");
  display.display();
}

void drawDIScreen(bool estopActive) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawHeader("PROBAR DI", estopActive);
  display.setCursor(0, 16);
  display.println("Prueba correcto");
  display.println("funcionamiento de");
  display.println("Digital Inputs");
  display.println("CH00-CH06");
  display.display();
}

void drawStatusScreen(bool estopActive) {
  uint8_t pmcState, pmcFlags, pmcActiveSource, selectedSource;
  bool pmcValid;

  portENTER_CRITICAL(&mux);
  pmcState = g_pmcRunState;
  pmcFlags = g_pmcFlags;
  pmcActiveSource = g_pmcActiveSource;
  pmcValid = g_pmcStatusValid;
  selectedSource = g_selectedControlSource;
  portEXIT_CRITICAL(&mux);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  drawHeader("ESTADO", estopActive);

  display.setCursor(0, 12);
  display.print("Sel: ");
  display.println(sourceToLong(selectedSource));

  display.print("Act: ");
  display.println(sourceToLong(pmcActiveSource));

  display.print("PMC: ");
  display.println(pmcValid ? robotStateToText(pmcState) : "SIN DATOS");

  display.setCursor(0, 38);
  display.print("Homed:");
  display.print((pmcFlags & PMC_FLAG_HOMED) ? "SI" : "NO");
  display.print(" Ena:");
  display.println((pmcFlags & PMC_FLAG_DRIVERS_ENABLED) ? "SI" : "NO");

  display.print("Busy:");
  display.print((pmcFlags & PMC_FLAG_BUSY) ? "SI" : "NO");
  display.print(" Err:");
  display.println((pmcFlags & PMC_FLAG_ERROR) ? "SI" : "NO");

  display.display();
}

// =====================================================
// MENU / NAVEGACION
// =====================================================
void updateMenuFromDpad(uint8_t dpad, uint16_t buttons) {
  uint32_t now = millis();
  if (now - lastMenuMoveMs < MENU_DEBOUNCE_MS) return;

  bool moved = false;
  uint8_t idx;
  portENTER_CRITICAL(&mux);
  idx = menuIndex;
  portEXIT_CRITICAL(&mux);

  if (dpad & DPAD_MASK_UP) {
    idx = (idx == 0) ? (MENU_COUNT - 1) : (idx - 1);
    moved = true;
  } else if (dpad & DPAD_MASK_DOWN) {
    idx = (idx + 1) % MENU_COUNT;
    moved = true;
  }

  if (moved) {
    lastMenuMoveMs = now;
    portENTER_CRITICAL(&mux);
    menuIndex = idx;
    portEXIT_CRITICAL(&mux);
  }

  if (buttons & BTN_SELECT) {
    lastMenuMoveMs = now;
    portENTER_CRITICAL(&mux);
    menuSelected = idx + 1;
    portEXIT_CRITICAL(&mux);

    if (idx == MENU_MANUAL) {
      portENTER_CRITICAL(&mux);
      g_mode = MODE_MANUAL_SELECT;
      menuSelected = 0;
      if (g_selectedControlSource == CONTROL_SOURCE_PC) {
        g_manualIndex = 2;
      } else if (g_manualMode == MANUAL_MODE_CARTESIAN) {
        g_manualIndex = 1;
      } else {
        g_manualIndex = 0;
      }
      g_lastOptionPressed = false;
      portEXIT_CRITICAL(&mux);
    } else if (idx == MENU_CAL) {
      portENTER_CRITICAL(&mux);
      g_mode = MODE_CAL_SELECT;
      g_calibrationIndex = 0;
      g_calibrationMode = CAL_MODE_NONE;
      menuSelected = 0;
      g_lastOptionPressed = false;
      portEXIT_CRITICAL(&mux);
    } else if (idx == MENU_DI) {
      setModeAndResetEstop(MODE_DI);
      portENTER_CRITICAL(&mux);
      menuSelected = idx + 1;
      portEXIT_CRITICAL(&mux);
    } else if (idx == MENU_AUTO) {
      setModeAndResetEstop(MODE_AUTO);
      portENTER_CRITICAL(&mux);
      g_selectedControlSource = CONTROL_SOURCE_AUTO;
      menuSelected = idx + 1;
      portEXIT_CRITICAL(&mux);
    } else if (idx == MENU_STATUS) {
      setModeAndResetEstop(MODE_STATUS);
      portENTER_CRITICAL(&mux);
      menuSelected = idx + 1;
      portEXIT_CRITICAL(&mux);
    } else if (idx == MENU_TEACH) {
      setModeAndResetEstop(MODE_TEACH);
      portENTER_CRITICAL(&mux);
      g_selectedControlSource = CONTROL_SOURCE_LOCAL;
      g_manualMode = MANUAL_MODE_JOINT;
      menuSelected = idx + 1;
      portEXIT_CRITICAL(&mux);
    }
  }
}

void updateCalibrationModeSelect(uint8_t dpad, uint16_t buttons, uint8_t misc) {
  if (misc & MISC_SHARE) {
    resetToMenu();
    return;
  }
  updateOptionToggle(misc);

  uint32_t now = millis();
  if (now - lastMenuMoveMs < MENU_DEBOUNCE_MS) return;

  uint8_t idx;
  portENTER_CRITICAL(&mux);
  idx = g_calibrationIndex;
  portEXIT_CRITICAL(&mux);

  if (dpad & DPAD_MASK_UP) {
    idx = (idx == 0) ? 3 : idx - 1;
    lastMenuMoveMs = now;
  } else if (dpad & DPAD_MASK_DOWN) {
    idx = (idx + 1) % 4;
    lastMenuMoveMs = now;
  }

  portENTER_CRITICAL(&mux);
  g_calibrationIndex = idx;
  portEXIT_CRITICAL(&mux);

  if (buttons & BTN_SELECT) {
    lastMenuMoveMs = now;
    portENTER_CRITICAL(&mux);
    g_calibrationMode = idx + 1;
    g_mode = MODE_CAL;
    menuSelected = MENU_CAL + 1;
    g_lastOptionPressed = false;
    portEXIT_CRITICAL(&mux);
  }
}

void updateManualModeSelect(uint8_t dpad, uint16_t buttons, uint8_t misc) {
  uint32_t now = millis();

  if (misc & MISC_SHARE) {
    resetToMenu();
    return;
  }

  updateOptionToggle(misc);

  uint8_t idx;
  portENTER_CRITICAL(&mux);
  idx = g_manualIndex;
  portEXIT_CRITICAL(&mux);

  if (now - lastMenuMoveMs >= MENU_DEBOUNCE_MS) {
    bool moved = false;
    if (dpad & DPAD_MASK_UP) {
      idx = (idx == 0) ? 2 : (idx - 1);
      moved = true;
    } else if (dpad & DPAD_MASK_DOWN) {
      idx = (idx + 1) % 3;
      moved = true;
    }

    if (moved) {
      lastMenuMoveMs = now;
      portENTER_CRITICAL(&mux);
      g_manualIndex = idx;
      portEXIT_CRITICAL(&mux);
    }

    if (buttons & BTN_SELECT) {
      lastMenuMoveMs = now;
      portENTER_CRITICAL(&mux);
      if (idx == 0) {
        g_selectedControlSource = CONTROL_SOURCE_LOCAL;
        g_manualMode = MANUAL_MODE_JOINT;
        menuSelected = MENU_MANUAL + 1;
        g_mode = MODE_MANUAL;
      } else if (idx == 1) {
        g_selectedControlSource = CONTROL_SOURCE_LOCAL;
        g_manualMode = MANUAL_MODE_CARTESIAN;
        menuSelected = MENU_MANUAL + 1;
        g_mode = MODE_MANUAL;
      } else {
        g_selectedControlSource = CONTROL_SOURCE_PC;
        g_manualMode = MANUAL_MODE_NONE;
        g_mode = MODE_MENU;
        menuSelected = MENU_MANUAL + 1;
      }
      g_lastOptionPressed = false;
      portEXIT_CRITICAL(&mux);
    }
  }
}

void updateSimpleScreen(uint8_t misc) {
  if (misc & MISC_SHARE) {
    resetToMenu();
    return;
  }
  updateOptionToggle(misc);
}

// =====================================================
// GAMEPAD
// =====================================================
void processGamepad(ControllerPtr ctl) {
  uint16_t M_Boton = ctl->buttons();
  const bool squarePressed = (M_Boton & BTN_SQUARE) != 0;
  bool squareJustPressed = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      squareJustPressed = squarePressed && !squareWasPressed[i];
      // Registrar tambien fuera de manual y durante E-stop: no diferir cierres.
      squareWasPressed[i] = squarePressed;
      break;
    }
  }
  uint8_t  M_Cruz  = ctl->dpad();
  int8_t   M_JAxisLx = mapAxisToInt8(ctl->axisX());
  int8_t   M_JAxisLy = mapAxisToInt8(ctl->axisY());
  int8_t   M_JAxisRx = mapAxisToInt8(ctl->axisRX());
  int8_t   M_JAxisRy = mapAxisToInt8(ctl->axisRY());
  uint8_t  M_L2 = mapTrigToUint8(ctl->brake());
  uint8_t  M_R2 = mapTrigToUint8(ctl->throttle());
  uint8_t  M_Misc = (uint8_t)ctl->miscButtons();

  portENTER_CRITICAL(&mux);
  Boton[0] = M_Boton;
  Cruz[0] = M_Cruz;
  JAxisLx[0] = M_JAxisLx;
  JAxisLy[0] = M_JAxisLy;
  JAxisRx[0] = M_JAxisRx;
  JAxisRy[0] = M_JAxisRy;
  L2[0] = M_L2;
  R2[0] = M_R2;
  Misc[0] = M_Misc;
  portEXIT_CRITICAL(&mux);

  uint8_t mode;
  portENTER_CRITICAL(&mux);
  mode = g_mode;
  portEXIT_CRITICAL(&mux);

  if (mode == MODE_MENU) {
    updateMenuFromDpad(M_Cruz, M_Boton);
    return;
  }

  if (mode == MODE_MANUAL_SELECT) {
    updateManualModeSelect(M_Cruz, M_Boton, M_Misc);
    return;
  }

  if (mode == MODE_CAL_SELECT) {
    updateCalibrationModeSelect(M_Cruz, M_Boton, M_Misc);
    return;
  }

  if (mode == MODE_CAL || mode == MODE_DI || mode == MODE_AUTO || mode == MODE_STATUS) {
    updateSimpleScreen(M_Misc);
    return;
  }

  if (mode == MODE_TEACH) {
    if (M_Misc & MISC_SHARE) {
      resetToMenu();
      return;
    }
    updateOptionToggle(M_Misc);
    return;
  }

  if (mode == MODE_MANUAL) {
    if (M_Misc & MISC_SHARE) {
      resetToMenu();
      return;
    }

    updateOptionToggle(M_Misc);

    bool estopNow;
    portENTER_CRITICAL(&mux);
    estopNow = g_estop;
    portEXIT_CRITICAL(&mux);

    if (estopNow) {
      return;
    }

    bool gripperChanged = false;
    if (squareJustPressed) {
      servo180Angle = GRIPPER_CLOSED_ANGLE;
      gripperChanged = true;
      lastGripperAdjustMs = millis();
      Serial.print("GRIPPER_MANUAL_CLOSE_CMD=");
      Serial.println(servo180Angle);
    } else if (!squarePressed && millis() - lastGripperAdjustMs >= 60) {
      if (M_Boton & BTN_L1) {
        servo180Angle--;
        gripperChanged = true;
      } else if (M_Boton & BTN_R1) {
        servo180Angle++;
        gripperChanged = true;
      }
      if (gripperChanged) lastGripperAdjustMs = millis();
    }

    servo180Angle = clampGripperAngle(servo180Angle);
    writeGripperServos(servo180Angle);

    if (gripperChanged) {
      Serial.print("GRIPPER_TEST_ANGLE=");
      Serial.println(servo180Angle);
    }

    bool rotationChanged = false;
    if (millis() - lastRotationAdjustMs >= 60) {
      if (M_Boton & BTN_L2) {
        servoRotationAngle--;
        rotationChanged = true;
      } else if (M_Boton & BTN_R2) {
        servoRotationAngle++;
        rotationChanged = true;
      }
      if (rotationChanged) lastRotationAdjustMs = millis();
    }

    servoRotationAngle = clampRotationAngle(servoRotationAngle);
    writeRotationServo(servoRotationAngle);

    if (rotationChanged) {
      Serial.print("ROTATION_TEST_ANGLE=");
      Serial.println(servoRotationAngle);
    }
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// =====================================================
// I2C ESCLAVO
// =====================================================
// Protocolo nuevo adicional:
// - La PMC puede escribir: ['S', runState, pmcFlags, activeSource]
//   para que la ESP muestre estado en la OLED.
// - La PMC puede escribir: ['G', angle] para controlar la pinza.
// - La PMC puede escribir: ['R', angle] para controlar el giro (0..180 grados).
// - El comando 'P' devuelve: [cmd, selectedSource, pmcRunState, pmcFlags, pmcActiveSource]
// - El comando 'M' sigue devolviendo 5 bytes por compatibilidad:
//   [cmd, menuState, manualMode, misc, flags]
//   flags:
//     bit0 = estop ESP
//     bit1..bit3 = fuente seleccionada (one-hot: local/pc/auto)
//     bit4 = PMC status valid
//     bit5 = PMC estop
//     bit6 = PMC busy
//     bit7 = PMC homed
void onReceive(int len) {
  if (len <= 0) return;

  uint8_t cmd = 0;
  if (Wire.available()) cmd = (uint8_t)Wire.read();

  if (cmd == 'S') {
    uint8_t runState = ROBOT_STATE_UNKNOWN;
    uint8_t pmcFlags = 0;
    uint8_t activeSource = CONTROL_SOURCE_LOCAL;

    if (Wire.available()) runState = (uint8_t)Wire.read();
    if (Wire.available()) pmcFlags = (uint8_t)Wire.read();
    if (Wire.available()) activeSource = (uint8_t)Wire.read();
    while (Wire.available()) (void)Wire.read();

    portENTER_CRITICAL(&mux);
    g_pmcRunState = runState;
    g_pmcFlags = pmcFlags;
    g_pmcActiveSource = activeSource;
    g_pmcStatusValid = true;
    portEXIT_CRITICAL(&mux);
    return;
  }

  if (cmd == 'G') {
    uint8_t angle = GRIPPER_OPEN_ANGLE;
    if (Wire.available()) angle = (uint8_t)Wire.read();
    while (Wire.available()) (void)Wire.read();
    angle = (uint8_t)clampGripperAngle((int)angle);

    portENTER_CRITICAL(&mux);
    servo180Angle = angle;
    g_gripperCommandPending = true;
    portEXIT_CRITICAL(&mux);
    return;
  }

  if (cmd == 'R') {
    uint8_t angle = ROTATION_INITIAL_ANGLE;
    if (Wire.available()) angle = (uint8_t)Wire.read();
    while (Wire.available()) (void)Wire.read();
    angle = (uint8_t)clampRotationAngle((int)angle);

    portENTER_CRITICAL(&mux);
    servoRotationAngle = angle;
    g_rotationCommandPending = true;
    portEXIT_CRITICAL(&mux);
    return;
  }

  while (Wire.available()) (void)Wire.read();
  portENTER_CRITICAL(&mux);
  lastCmd = cmd;
  portEXIT_CRITICAL(&mux);
}

void onRequest() {
  uint8_t cmd;
  uint16_t buttons;
  uint8_t dpad, lt, rt, misc;
  int8_t lx, ly, rx, ry;
  uint8_t menuState;
  uint8_t manualMode;
  uint8_t calibrationMode;
  uint8_t selectedSource;
  bool estop;
  bool pmcStatusValid;
  uint8_t pmcRunState;
  uint8_t pmcFlags;
  uint8_t pmcActiveSource;

  portENTER_CRITICAL(&mux);
  cmd = lastCmd;
  buttons = Boton[0];
  dpad = Cruz[0];
  lx = JAxisLx[0];
  ly = JAxisLy[0];
  rx = JAxisRx[0];
  ry = JAxisRy[0];
  lt = L2[0];
  rt = R2[0];
  misc = Misc[0];
  menuState = menuSelected;
  manualMode = g_manualMode;
  calibrationMode = g_calibrationMode;
  selectedSource = g_selectedControlSource;
  estop = g_estop;
  pmcStatusValid = g_pmcStatusValid;
  pmcRunState = g_pmcRunState;
  pmcFlags = g_pmcFlags;
  pmcActiveSource = g_pmcActiveSource;
  portEXIT_CRITICAL(&mux);

  uint8_t flags = 0;
  if (estop) flags |= 0x01;
  if (selectedSource == CONTROL_SOURCE_LOCAL) flags |= 0x02;
  if (selectedSource == CONTROL_SOURCE_PC)    flags |= 0x04;
  if (selectedSource == CONTROL_SOURCE_AUTO)  flags |= 0x08;
  if (pmcStatusValid) flags |= 0x10;
  if (pmcFlags & PMC_FLAG_ESTOP) flags |= 0x20;
  if (pmcFlags & PMC_FLAG_BUSY)  flags |= 0x40;
  if (pmcFlags & PMC_FLAG_HOMED) flags |= 0x80;

  switch (cmd) {
    case 'B': {
      uint8_t out[3] = { cmd, (uint8_t)(buttons & 0xFF), (uint8_t)(buttons >> 8) };
      Wire.write(out, sizeof(out));
    } break;

    case 'C': {
      uint8_t out[2] = { cmd, dpad };
      Wire.write(out, sizeof(out));
    } break;

    case 'J': {
      uint8_t out[5] = { cmd, (uint8_t)lx, (uint8_t)ly, (uint8_t)rx, (uint8_t)ry };
      Wire.write(out, sizeof(out));
    } break;

    case 'T': {
      uint8_t out[3] = { cmd, lt, rt };
      Wire.write(out, sizeof(out));
    } break;

    case 'P': {
      uint8_t out[5] = { cmd, selectedSource, pmcRunState, pmcFlags, pmcActiveSource };
      Wire.write(out, sizeof(out));
    } break;

    case 'M': {
      uint8_t subMode = (menuState == MENU_CAL + 1) ? calibrationMode : manualMode;
      uint8_t out[5] = { cmd, menuState, subMode, misc, flags };
      Wire.write(out, sizeof(out));
    } break;

    default: {
      Wire.write(&cmd, 1);
    } break;
  }
}

// =====================================================
// DIAGNOSTICO I2C OLED BUS
// =====================================================
void scanWireOLED() {
  Serial.println("\nEscaneando bus I2C WireOLED (SDA GPIO 21 / SCL GPIO 22)...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    WireOLED.beginTransmission(addr);
    uint8_t err = WireOLED.endTransmission();
    if (err == 0) {
      Serial.printf("Dispositivo encontrado en 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("No se encontraron dispositivos en WireOLED");
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  servo180.attach(SERVO_GRIPPER_1_PIN);
  servoSecond.attach(SERVO_GRIPPER_2_PIN);
  servoRotation.attach(SERVO_ROTATION_PIN);
  writeGripperServos(servo180Angle);
  writeRotationServo(servoRotationAngle);

  // I2C esclavo para la Portenta
  Wire.begin((uint8_t)I2C_ADDR, (int)SDA_PIN, (int)SCL_PIN, (uint32_t)I2C_FREQ);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  // OLED en bus separado
  WireOLED.begin(OLED_SDA, OLED_SCL, 50000);
  WireOLED.setClock(50000);
  // reset=true genera el pulso LOW/HIGH por GPIO 26.
  // periphBegin=false evita que Adafruit_SSD1306 reinicialice WireOLED
  // y cambie los pines personalizados 21/22.
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) {
    Serial.println("OLED no detectada");
  } else {
    WireOLED.setClock(50000);
    drawMenu(0, 0, false);
  }

  delay(500);
  scanWireOLED();

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("ESP32 WROOM-32 0x28 listo: Bluepad32 + I2C + OLED + 3 servos + GUI PC");
}

void loop() {
  bool applyGripperCommand = false;
  bool applyRotationCommand = false;
  int commandedGripperAngle = GRIPPER_OPEN_ANGLE;
  int commandedRotationAngle = ROTATION_INITIAL_ANGLE;
  portENTER_CRITICAL(&mux);
  if (g_gripperCommandPending) {
    commandedGripperAngle = servo180Angle;
    g_gripperCommandPending = false;
    applyGripperCommand = true;
  }
  if (g_rotationCommandPending) {
    commandedRotationAngle = servoRotationAngle;
    g_rotationCommandPending = false;
    applyRotationCommand = true;
  }
  portEXIT_CRITICAL(&mux);

  if (applyGripperCommand) {
    commandedGripperAngle = clampGripperAngle(commandedGripperAngle);
    writeGripperServos(commandedGripperAngle);
    Serial.print("GRIPPER_I2C_ANGLE=");
    Serial.println(commandedGripperAngle);
  }

  if (applyRotationCommand) {
    commandedRotationAngle = clampRotationAngle(commandedRotationAngle);
    writeRotationServo(commandedRotationAngle);
    Serial.print("ROTATION_I2C_ANGLE=");
    Serial.println(commandedRotationAngle);
  }

  if (BP32.update()) {
    processControllers();
  }

  // refrescar OLED
  static uint32_t lastOledMs = 0;
  uint8_t modeCheck;
  portENTER_CRITICAL(&mux);
  modeCheck = g_mode;
  portEXIT_CRITICAL(&mux);

  uint32_t oledPeriod = (modeCheck == MODE_AUTO) ? 500 : 150;

  if (millis() - lastOledMs > oledPeriod) {
    lastOledMs = millis();

    uint8_t idx, sel, mode, manualMode, manualIndex, calibrationMode, calibrationIndex;
    bool estop, calDone;

    portENTER_CRITICAL(&mux);
    idx = menuIndex;
    sel = menuSelected;
    mode = g_mode;
    manualMode = g_manualMode;
    manualIndex = g_manualIndex;
    calibrationMode = g_calibrationMode;
    calibrationIndex = g_calibrationIndex;
    estop = g_estop;
    calDone = false;
    portEXIT_CRITICAL(&mux);

    if (g_pmcStatusValid && !(g_pmcFlags & PMC_FLAG_BUSY) &&
        g_pmcRunState == ROBOT_STATE_READY && mode == MODE_CAL) calDone = true;

    if (mode == MODE_CAL_SELECT) {
      drawCalibrationSelectScreen(calibrationIndex, estop);
    } else if (mode == MODE_MANUAL_SELECT) {
      drawManualSelectScreen(manualIndex, estop);
    } else if (mode == MODE_MANUAL) {
      drawManualScreen(estop, manualMode);
    } else if (mode == MODE_CAL) {
      drawCalibrationScreen(estop, calDone, calibrationMode);
    } else if (mode == MODE_AUTO) {
      drawAutoScreen(estop);
    } else if (mode == MODE_DI) {
      drawDIScreen(estop);
    } else if (mode == MODE_STATUS) {
      drawStatusScreen(estop);
    } else if (mode == MODE_TEACH) {
      drawTeachScreen(estop);
    } else {
      drawMenu(idx, sel, estop);
    }
  }

  // Log serial opcional
  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = millis();

    uint16_t buttons;
    uint8_t dpad, lt, rt, cmd, selSource, pmcState, pmcFlags, pmcSrc;
    int8_t lx, ly, rx, ry;
    uint8_t msel, mmode;
    bool estop, pmcValid;

    portENTER_CRITICAL(&mux);
    cmd = lastCmd;
    buttons = Boton[0];
    dpad = Cruz[0];
    lx = JAxisLx[0];
    ly = JAxisLy[0];
    rx = JAxisRx[0];
    ry = JAxisRy[0];
    lt = L2[0];
    rt = R2[0];
    msel = menuSelected;
    mmode = g_manualMode;
    estop = g_estop;
    selSource = g_selectedControlSource;
    pmcValid = g_pmcStatusValid;
    pmcState = g_pmcRunState;
    pmcFlags = g_pmcFlags;
    pmcSrc = g_pmcActiveSource;
    portEXIT_CRITICAL(&mux);

    Serial.printf(
      "CMD=%c MENU=%u MMODE=%u SRC=%u ESTOP=%u MISC=0x%02X BTN=0x%04X DPAD=0x%02X LX=%d LY=%d RX=%d RY=%d LT=%u RT=%u PMCvalid=%u PMCstate=%u PMCflags=0x%02X PMCsrc=%u\n",
      cmd, (unsigned)msel, (unsigned)mmode, (unsigned)selSource, (unsigned)estop,
      (unsigned)Misc[0], buttons, dpad, lx, ly, rx, ry, lt, rt,
      (unsigned)pmcValid, (unsigned)pmcState, (unsigned)pmcFlags, (unsigned)pmcSrc
    );
  }

  // Esta ESP WROOM-32 (0x28) ya no maneja la HuskyLens.
  // La vision queda en la ESP WROOM 32 dedicada con direccion 0x29.

  vTaskDelay(1);
}
