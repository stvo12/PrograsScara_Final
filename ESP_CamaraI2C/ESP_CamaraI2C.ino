#include <Wire.h>
#include "DFRobot_HuskylensV2.h"
#include <math.h>
#include "esp_bt.h"

// =====================================================
// I2C ESCLAVO HACIA PMC
// =====================================================
#define HUSKY_ESP32_ADDR 0x29
#define PMC_I2C_SDA 25
#define PMC_I2C_SCL 26

// =====================================================
// UART HACIA HUSKYLENS
// =====================================================
#define HUSKY_UART_RX 21
#define HUSKY_UART_TX 22
#define HUSKY_UART_BAUD 115200

HardwareSerial HuskySerial(1);
HuskylensV2 huskylens;

#define ALGORITHM_MODELO_PRUEBA_V2 ((eAlgorithm_t)131)

static const uint32_t HUSKY_READ_PERIOD_MS = 500;
static const uint32_t HUSKY_ACTIVE_TIMEOUT_MS = 2000;

// flags:
// bit0 = Husky inicializada
// bit1 = modelo seleccionado
// bit2 = pieza detectada
// bit3 = ultima lectura OK
// bit4 = error
// bit5 = conversion pixel -> mm valida
// bit6 = nombre de pieza valido (1..16 caracteres)
volatile uint8_t g_huskyFlags = 0;
volatile char g_huskyName[17] = {0};
volatile uint8_t g_huskyCount = 0;
volatile int16_t g_huskyX = 0;
volatile int16_t g_huskyY = 0;
volatile int16_t g_huskyW = 0;
volatile int16_t g_huskyH = 0;

enum HuskyMode {
  HUSKY_IDLE,
  HUSKY_CALIB_TAGS,
  HUSKY_DETECT_PIECES
};

volatile HuskyMode g_huskyMode = HUSKY_IDLE;

volatile bool g_visualCalibrated = false;
volatile int16_t g_camXMin = 0;
volatile int16_t g_camXMax = 0;
volatile int16_t g_camYMin = 0;
volatile int16_t g_camYMax = 0;

// =====================================================
// MAPEO PIXEL -> MM MEDIANTE 4 APRILTAGS
// =====================================================
//
// Tag 1 -------- Tag 2
//   |              |
//   |    248 mm    |
//   |              |
// Tag 3 -------- Tag 4
//       424 mm
//
// Tag1 = (0,0)
// Tag2 = (424,0)
// Tag3 = (0,248)
// Tag4 = (424,248)

static constexpr float BOARD_WIDTH_MM = 424.0f;
static constexpr float BOARD_HEIGHT_MM = 248.0f;

volatile int16_t g_tagX[4] = {0, 0, 0, 0};
volatile int16_t g_tagY[4] = {0, 0, 0, 0};

float g_pixelToBoardH[8] = {0};
volatile bool g_pixelMapValid = false;

volatile int16_t g_pieceXmm10 = 0;
volatile int16_t g_pieceYmm10 = 0;

// flags calibracion:
// bit0 = Husky inicializada
// bit1 = Tag Recognition seleccionado
// bit2 = tag 1
// bit3 = tag 2
// bit4 = tag 3
// bit5 = tag 4
// bit6 = error
// bit7 = calibracion completa
volatile uint8_t g_calibFlags = 0;

volatile uint8_t lastCmd = 'N';
volatile bool g_autoActive = false;
volatile uint32_t lastHuskyRequestMs = 0;

static bool huskyInitDone = false;
static bool huskyModelSelected = false;
static bool huskyTagModeSelected = false;

static uint32_t lastReadMs = 0;
static uint32_t lastInitAttemptMs = 0;
static uint32_t lastTagReadMs = 0;
static uint8_t huskyReadErrorCount = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// DATOS HUSKY
// =====================================================

void clearHuskyData(uint8_t flags = 0) {
  portENTER_CRITICAL(&mux);
  g_huskyFlags = flags;
  for (uint8_t i = 0; i < sizeof(g_huskyName); i++) g_huskyName[i] = 0;
  g_huskyCount = 0;
  g_huskyX = 0;
  g_huskyY = 0;
  g_huskyW = 0;
  g_huskyH = 0;
  g_pieceXmm10 = 0;
  g_pieceYmm10 = 0;
  portEXIT_CRITICAL(&mux);
}

void setHuskyNoDetection(uint8_t flags) {
  portENTER_CRITICAL(&mux);
  g_huskyFlags = flags;
  for (uint8_t i = 0; i < sizeof(g_huskyName); i++) g_huskyName[i] = 0;
  g_huskyCount = 0;
  g_huskyX = 0;
  g_huskyY = 0;
  g_huskyW = 0;
  g_huskyH = 0;
  g_pieceXmm10 = 0;
  g_pieceYmm10 = 0;
  portEXIT_CRITICAL(&mux);
}

void setHuskyDetection(uint8_t flags, const String& name, uint8_t count,
                       int16_t x, int16_t y, int16_t w, int16_t h) {
  const size_t nameLength = name.length();
  const bool nameValid = nameLength > 0 && nameLength < sizeof(g_huskyName);
  portENTER_CRITICAL(&mux);
  g_huskyFlags = nameValid ? (flags | 0x40) : (flags & (uint8_t)~0x40);
  for (uint8_t i = 0; i < sizeof(g_huskyName); i++) {
    g_huskyName[i] = (nameValid && i < nameLength) ? name[i] : 0;
  }
  g_huskyCount = count;
  g_huskyX = x;
  g_huskyY = y;
  g_huskyW = w;
  g_huskyH = h;
  portEXIT_CRITICAL(&mux);
}

// =====================================================
// HOMOGRAFIA
// =====================================================

static bool solveLinear8(float a[8][9], float out[8]) {
  for (int col = 0; col < 8; ++col) {
    int pivot = col;
    float best = fabsf(a[col][col]);

    for (int row = col + 1; row < 8; ++row) {
      float v = fabsf(a[row][col]);
      if (v > best) {
        best = v;
        pivot = row;
      }
    }

    if (best < 1.0e-6f) return false;

    if (pivot != col) {
      for (int k = col; k < 9; ++k) {
        float temp = a[col][k];
        a[col][k] = a[pivot][k];
        a[pivot][k] = temp;
      }
    }

    float div = a[col][col];
    for (int k = col; k < 9; ++k) a[col][k] /= div;

    for (int row = 0; row < 8; ++row) {
      if (row == col) continue;
      float factor = a[row][col];
      for (int k = col; k < 9; ++k) a[row][k] -= factor * a[col][k];
    }
  }

  for (int i = 0; i < 8; ++i) out[i] = a[i][8];
  return true;
}

static bool buildPixelToBoardHomography(const int16_t px[4], const int16_t py[4]) {
  const float realX[4] = {0.0f, BOARD_WIDTH_MM, 0.0f, BOARD_WIDTH_MM};
  const float realY[4] = {0.0f, 0.0f, BOARD_HEIGHT_MM, BOARD_HEIGHT_MM};

  float a[8][9] = {};

  for (int i = 0; i < 4; ++i) {
    float u = (float)px[i];
    float v = (float)py[i];
    int r = 2 * i;

    a[r][0] = u;
    a[r][1] = v;
    a[r][2] = 1.0f;
    a[r][6] = -realX[i] * u;
    a[r][7] = -realX[i] * v;
    a[r][8] = realX[i];

    a[r + 1][3] = u;
    a[r + 1][4] = v;
    a[r + 1][5] = 1.0f;
    a[r + 1][6] = -realY[i] * u;
    a[r + 1][7] = -realY[i] * v;
    a[r + 1][8] = realY[i];
  }

  float h[8];
  if (!solveLinear8(a, h)) return false;

  for (int i = 0; i < 8; ++i) g_pixelToBoardH[i] = h[i];
  return true;
}

static bool pixelToBoardMm(float pixelX, float pixelY, float& xMm, float& yMm) {
  if (!g_pixelMapValid) return false;

  const float* h = g_pixelToBoardH;
  float denominator = h[6] * pixelX + h[7] * pixelY + 1.0f;

  if (fabsf(denominator) < 1.0e-6f) return false;

  xMm = (h[0] * pixelX + h[1] * pixelY + h[2]) / denominator;
  yMm = (h[3] * pixelX + h[4] * pixelY + h[5]) / denominator;

  const float margin = 10.0f;

  return xMm >= -margin &&
         xMm <= BOARD_WIDTH_MM + margin &&
         yMm >= -margin &&
         yMm <= BOARD_HEIGHT_MM + margin;
}

// =====================================================
// AUTO
// =====================================================

void updateHuskyAuto(bool active) {
  if (!active) {
    clearHuskyData(0);
    return;
  }

  if (!huskyInitDone) {
    if (millis() - lastInitAttemptMs < 1000) return;
    lastInitAttemptMs = millis();

    Serial.println("Inicializando HuskyLens...");

    if (!huskylens.begin(HuskySerial)) {
      Serial.println("HuskyLens no detectada.");
      clearHuskyData(0x10);
      return;
    }

    huskylens.timeOutDuration = 300;
    huskylens.retry = 1;

    Serial.println("HuskyLens detectada.");
    huskyInitDone = true;
  }

  uint8_t flags = 0x01;

  if (!huskyModelSelected) {
    Serial.println("Cambiando a Modelo UnaClase...");

    if (!huskylens.switchAlgorithm(ALGORITHM_MODELO_PRUEBA_V2)) {
      Serial.println("No se pudo cambiar a Modelo UnaClase.");
      clearHuskyData(flags | 0x10);
      return;
    }

    Serial.println("Modelo UnaClase seleccionado.");

    huskyModelSelected = true;
    huskyTagModeSelected = false;
    huskyReadErrorCount = 0;

    delay(1500);

    lastReadMs = millis();

    Serial.println("Esperando estabilizacion del modelo...");
    return;
  }

  flags |= 0x02;

  if (millis() - lastReadMs < HUSKY_READ_PERIOD_MS) return;
  lastReadMs = millis();

  Serial.println("[HUSKY] Antes de getResult");

  int8_t count = huskylens.getResult(ALGORITHM_MODELO_PRUEBA_V2);

  Serial.print("[HUSKY] Despues de getResult, count=");
  Serial.println(count);

  if (count < 0) {
    huskyReadErrorCount++;

    Serial.print("Error leyendo HuskyLens. Fallos consecutivos = ");
    Serial.println(huskyReadErrorCount);

    clearHuskyData(flags | 0x10);
    return;
  }

  huskyReadErrorCount = 0;

  if (count == 0) {
    setHuskyNoDetection(flags | 0x08);
    Serial.println("Husky OK, sin pieza.");
    return;
  }

  if (!huskylens.available(ALGORITHM_MODELO_PRUEBA_V2)) {
    setHuskyNoDetection(flags | 0x08);
    Serial.println("Conteo recibido, pero sin cache disponible.");
    return;
  }

  Result *result = static_cast<Result *>(
    huskylens.getCachedCenterResult(ALGORITHM_MODELO_PRUEBA_V2)
  );

  if (result == NULL) {
    clearHuskyData(flags | 0x10);
    Serial.println("Resultado nulo.");
    return;
  }

  flags |= 0x04;
  flags |= 0x08;

  setHuskyDetection(
    flags,
    result->name,
    (uint8_t)count,
    result->xCenter,
    result->yCenter,
    result->width,
    result->height
  );

  float boardXmm = 0.0f;
  float boardYmm = 0.0f;

  bool mapped = pixelToBoardMm(
    result->xCenter,
    result->yCenter,
    boardXmm,
    boardYmm
  );

  portENTER_CRITICAL(&mux);

  if (mapped) {
    g_pieceXmm10 = (int16_t)lroundf(boardXmm * 10.0f);
    g_pieceYmm10 = (int16_t)lroundf(boardYmm * 10.0f);
    g_huskyFlags |= 0x20;
  } else {
    g_pieceXmm10 = 0;
    g_pieceYmm10 = 0;
    g_huskyFlags &= (uint8_t)~0x20;
  }

  portEXIT_CRITICAL(&mux);

  if (mapped) {
    Serial.print("Posicion fisica [mm] = (");
    Serial.print(boardXmm, 1);
    Serial.print(", ");
    Serial.print(boardYmm, 1);
    Serial.println(")");
  } else {
    Serial.println("Mapeo pixel -> mm invalido.");
  }

  Serial.print("Pieza detectada nombre='");
  Serial.print(result->name);
  Serial.print("' ID_HUSKY_DIAGNOSTICO=");
  Serial.print(result->ID);
  Serial.print(" Centro=(");
  Serial.print(result->xCenter);
  Serial.print(", ");
  Serial.print(result->yCenter);
  Serial.print(") Size=(");
  Serial.print(result->width);
  Serial.print(", ");
  Serial.print(result->height);
  Serial.println(")");
}

// =====================================================
// I2C SLAVE CALLBACKS PMC
// =====================================================

void onReceiveFromPmc(int len) {
  if (len <= 0) return;

  uint8_t cmd = Wire.read();
  while (Wire.available()) (void)Wire.read();

  uint32_t now = millis();

  if (cmd == 'N') {
    portENTER_CRITICAL(&mux);

    lastCmd = cmd;
    g_autoActive = true;
    lastHuskyRequestMs = now;

    if (g_huskyMode != HUSKY_DETECT_PIECES) {
      g_huskyMode = HUSKY_DETECT_PIECES;
      // Al regresar de calibracion no publicar una pieza del modo anterior.
      g_huskyFlags = 0;
      g_huskyCount = 0;
      for (uint8_t i = 0; i < sizeof(g_huskyName); i++) g_huskyName[i] = 0;
      g_huskyX = g_huskyY = g_huskyW = g_huskyH = 0;
      g_pieceXmm10 = g_pieceYmm10 = 0;
      // Un timeout solo pausa las lecturas: la HuskyLens sigue en el mismo
      // algoritmo. Forzar false aqui repetia innecesariamente el cambio de
      // modelo despues de cada movimiento bloqueante de la Portenta.
      if (huskyTagModeSelected) huskyModelSelected = false;
      lastReadMs = 0;
    }

    portEXIT_CRITICAL(&mux);
    return;
  }

  if (cmd == 'C') {
    portENTER_CRITICAL(&mux);

    lastCmd = cmd;
    g_huskyMode = HUSKY_CALIB_TAGS;
    g_visualCalibrated = false;
    g_pixelMapValid = false;
    g_calibFlags = 0;
    g_camXMin = 0;
    g_camXMax = 0;
    g_camYMin = 0;
    g_camYMax = 0;
    for (uint8_t i = 0; i < 4; i++) {
      g_tagX[i] = 0;
      g_tagY[i] = 0;
    }
    g_autoActive = false;
    g_huskyFlags = 0;
    g_huskyCount = 0;
    for (uint8_t i = 0; i < sizeof(g_huskyName); i++) g_huskyName[i] = 0;
    g_huskyX = g_huskyY = g_huskyW = g_huskyH = 0;

    huskyTagModeSelected = false;
    huskyModelSelected = false;
    lastReadMs = 0;
    lastTagReadMs = 0;
    huskyReadErrorCount = 0;
    g_pieceXmm10 = 0;
    g_pieceYmm10 = 0;

    portEXIT_CRITICAL(&mux);
    return;
  }

  portENTER_CRITICAL(&mux);
  lastCmd = cmd;
  portEXIT_CRITICAL(&mux);
}

void onRequestFromPmc() {
  uint8_t cmd, flags, count;
  int16_t x, y, w, h, xMm10, yMm10;
  char name[17];

  portENTER_CRITICAL(&mux);

  cmd = lastCmd;
  flags = g_huskyFlags;
  for (uint8_t i = 0; i < sizeof(name); i++) name[i] = g_huskyName[i];
  count = g_huskyCount;
  x = g_huskyX;
  y = g_huskyY;
  w = g_huskyW;
  h = g_huskyH;
  xMm10 = g_pieceXmm10;
  yMm10 = g_pieceYmm10;

  portEXIT_CRITICAL(&mux);

  switch (cmd) {
    case 'N': {
      // 32 bytes: eco, flags, count, 6 coordenadas int16 y nombre[17].
      // El nombre se transporta literalmente; el ID no decide el destino.
      uint8_t out[32] = {
        cmd,
        flags,
        count,
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF),
        (uint8_t)(w & 0xFF), (uint8_t)((w >> 8) & 0xFF),
        (uint8_t)(h & 0xFF), (uint8_t)((h >> 8) & 0xFF),
        (uint8_t)(xMm10 & 0xFF), (uint8_t)((xMm10 >> 8) & 0xFF),
        (uint8_t)(yMm10 & 0xFF), (uint8_t)((yMm10 >> 8) & 0xFF)
      };
      for (uint8_t i = 0; i < sizeof(name); i++) out[15 + i] = (uint8_t)name[i];
      Wire.write(out, sizeof(out));
    } break;

    case 'K': {
      uint8_t calibFlags;
      int16_t xMin, xMax, yMin, yMax;

      portENTER_CRITICAL(&mux);

      calibFlags = g_calibFlags;
      xMin = g_camXMin;
      xMax = g_camXMax;
      yMin = g_camYMin;
      yMax = g_camYMax;

      portEXIT_CRITICAL(&mux);

      uint8_t out[10] = {
        cmd,
        calibFlags,
        (uint8_t)(xMin & 0xFF), (uint8_t)((xMin >> 8) & 0xFF),
        (uint8_t)(xMax & 0xFF), (uint8_t)((xMax >> 8) & 0xFF),
        (uint8_t)(yMin & 0xFF), (uint8_t)((yMin >> 8) & 0xFF),
        (uint8_t)(yMax & 0xFF), (uint8_t)((yMax >> 8) & 0xFF)
      };

      Wire.write(out, sizeof(out));
    } break;

    default:
      Wire.write(&cmd, 1);
      break;
  }
}

// =====================================================
// CALIBRACION APRILTAGS
// =====================================================

static uint8_t tagNumberFromText(const String& text) {
  if (text == "1") return 1;
  if (text == "2") return 2;
  if (text == "3") return 3;
  if (text == "4") return 4;
  return 0;
}

void updateHuskyTagCalibration() {
  if (!huskyInitDone) {
    if (millis() - lastInitAttemptMs < 1000) return;
    lastInitAttemptMs = millis();

    Serial.println("Inicializando HuskyLens para tags...");

    if (!huskylens.begin(HuskySerial)) {
      Serial.println("HuskyLens no detectada.");
      clearHuskyData(0x10);

      portENTER_CRITICAL(&mux);
      g_calibFlags = 0x40;
      portEXIT_CRITICAL(&mux);

      return;
    }

    huskylens.timeOutDuration = 300;
    huskylens.retry = 1;
    huskyInitDone = true;

    portENTER_CRITICAL(&mux);
    g_calibFlags |= 0x01;
    portEXIT_CRITICAL(&mux);

    Serial.println("HuskyLens detectada para tags.");
  }

  if (!huskyTagModeSelected) {
    Serial.println("Cambiando a Tag Recognition...");

    if (!huskylens.switchAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
      Serial.println("No se pudo cambiar a Tag Recognition.");

      portENTER_CRITICAL(&mux);
      g_calibFlags |= 0x40;
      portEXIT_CRITICAL(&mux);

      return;
    }

    huskyTagModeSelected = true;
    huskyModelSelected = false;

    portENTER_CRITICAL(&mux);
    g_calibFlags |= 0x02;
    portEXIT_CRITICAL(&mux);

    delay(800);
    lastTagReadMs = millis();
    return;
  }

  if (millis() - lastTagReadMs < 500) return;
  lastTagReadMs = millis();

  int8_t count = huskylens.getResult(ALGORITHM_TAG_RECOGNITION);

  if (count < 0) {
    Serial.println("Error leyendo tags.");

    portENTER_CRITICAL(&mux);
    g_calibFlags |= 0x40;
    portEXIT_CRITICAL(&mux);

    return;
  }

  if (count == 0) {
    Serial.println("Tag Recognition OK, pero no ve tags.");
    return;
  }

  bool found1 = false, found2 = false, found3 = false, found4 = false;

  int16_t x1 = 0, y1 = 0;
  int16_t x2 = 0, y2 = 0;
  int16_t x3 = 0, y3 = 0;
  int16_t x4 = 0, y4 = 0;

  for (int i = 0; i < count; i++) {
    Result *r = static_cast<Result *>(
      huskylens.getCachedResultByIndex(ALGORITHM_TAG_RECOGNITION, i)
    );

    if (r == NULL) continue;

    // El ID de aprendizaje puede ser 0; el contenido es el numero
    // codificado dentro del AprilTag. Usar el nombre solo si no hay contenido.
    uint8_t tagNumber = tagNumberFromText(r->content);
    if (tagNumber == 0 && r->content.length() == 0) {
      tagNumber = tagNumberFromText(r->name);
    }

    Serial.print("TAG id_aprendizaje=");
    Serial.print(r->ID);
    Serial.print(" nombre='");
    Serial.print(r->name);
    Serial.print("' contenido='");
    Serial.print(r->content);
    Serial.print("' numero=");
    Serial.print(tagNumber);
    Serial.print(" x=");
    Serial.print(r->xCenter);
    Serial.print(" y=");
    Serial.println(r->yCenter);

    switch (tagNumber) {
      case 1:
        found1 = true;
        x1 = r->xCenter;
        y1 = r->yCenter;
        break;

      case 2:
        found2 = true;
        x2 = r->xCenter;
        y2 = r->yCenter;
        break;

      case 3:
        found3 = true;
        x3 = r->xCenter;
        y3 = r->yCenter;
        break;

      case 4:
        found4 = true;
        x4 = r->xCenter;
        y4 = r->yCenter;
        break;
    }
  }

  uint8_t flags = 0x01 | 0x02;

  if (found1) flags |= 0x04;
  if (found2) flags |= 0x08;
  if (found3) flags |= 0x10;
  if (found4) flags |= 0x20;

  if (found1 && found2 && found3 && found4) {
    int16_t camXMin = (x1 + x3) / 2;
    int16_t camXMax = (x2 + x4) / 2;
    int16_t camYMin = (y1 + y2) / 2;
    int16_t camYMax = (y3 + y4) / 2;

    int16_t px[4] = {x1, x2, x3, x4};
    int16_t py[4] = {y1, y2, y3, y4};

    bool homographyOK = buildPixelToBoardHomography(px, py);

    if (!homographyOK) {
      Serial.println("ERROR: no se pudo calcular homografia.");

      portENTER_CRITICAL(&mux);
      g_calibFlags |= 0x40;
      g_pixelMapValid = false;
      portEXIT_CRITICAL(&mux);

      return;
    }

    flags |= 0x80;

    portENTER_CRITICAL(&mux);

    g_calibFlags = flags;

    g_camXMin = camXMin;
    g_camXMax = camXMax;
    g_camYMin = camYMin;
    g_camYMax = camYMax;

    g_visualCalibrated = true;
    g_huskyMode = HUSKY_IDLE;

    g_tagX[0] = x1;
    g_tagY[0] = y1;
    g_tagX[1] = x2;
    g_tagY[1] = y2;
    g_tagX[2] = x3;
    g_tagY[2] = y3;
    g_tagX[3] = x4;
    g_tagY[3] = y4;

    g_pixelMapValid = true;

    portEXIT_CRITICAL(&mux);

    Serial.println("Calibracion visual completada:");
    Serial.print("camXMin=");
    Serial.print(camXMin);
    Serial.print(" camXMax=");
    Serial.print(camXMax);
    Serial.print(" camYMin=");
    Serial.print(camYMin);
    Serial.print(" camYMax=");
    Serial.println(camYMax);

    huskyTagModeSelected = false;
    return;
  }

  portENTER_CRITICAL(&mux);
  g_calibFlags = flags;
  portEXIT_CRITICAL(&mux);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // --- DESACTIVAR BLUETOOTH DEFINITIVAMENTE ---
  btStop(); // Detener controlador BT
  esp_bt_controller_disable(); // Apagar hardware BT
  esp_bt_controller_deinit(); // Liberar memoria RAM del stack BT
  // --------------------------------------------
  
  Serial.begin(115200);
  delay(700);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" ESP WROOM32 HUSKYLENS UART / SLAVE 0x29 ");
  Serial.println("====================================");

  HuskySerial.begin(
      HUSKY_UART_BAUD,
      SERIAL_8N1,
      HUSKY_UART_RX,
      HUSKY_UART_TX
  );
  Serial.println("HuskyLens UART RX=21 TX=22 baud=115200.");

  Wire.begin((uint8_t)HUSKY_ESP32_ADDR, PMC_I2C_SDA, PMC_I2C_SCL, 100000);
  Wire.onReceive(onReceiveFromPmc);
  Wire.onRequest(onRequestFromPmc);

  clearHuskyData(0);

  Serial.println("ESP WROOM lista como I2C slave 0x29.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  HuskyMode modeLocal;
  bool autoActiveLocal;
  uint32_t lastReq;

  portENTER_CRITICAL(&mux);
  modeLocal = g_huskyMode;
  autoActiveLocal = g_autoActive;
  lastReq = lastHuskyRequestMs;
  portEXIT_CRITICAL(&mux);

  if (autoActiveLocal && (millis() - lastReq > HUSKY_ACTIVE_TIMEOUT_MS)) {
    portENTER_CRITICAL(&mux);

    g_autoActive = false;

    if (g_huskyMode == HUSKY_DETECT_PIECES) {
      g_huskyMode = HUSKY_IDLE;
    }

    portEXIT_CRITICAL(&mux);

    autoActiveLocal = false;
    modeLocal = HUSKY_IDLE;
  }

  switch (modeLocal) {
    case HUSKY_CALIB_TAGS:
      updateHuskyTagCalibration();
      break;

    case HUSKY_DETECT_PIECES:
      updateHuskyAuto(autoActiveLocal);
      break;

    case HUSKY_IDLE:
    default:
      delay(5);
      break;
  }

  delay(5);
}
