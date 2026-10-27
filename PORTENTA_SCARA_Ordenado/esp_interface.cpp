#include "esp_interface.h"

#include <Wire.h>
#include <string.h>
#include "project_config.h"
#include "robot_state.h"
#include "scara_core_bridge.h"

namespace {
static const uint8_t PMC_FLAG_ESTOP = 0x01;
static const uint8_t PMC_FLAG_HOMED = 0x02;
static const uint8_t PMC_FLAG_DRIVERS_ENABLED = 0x04;
static const uint8_t PMC_FLAG_BUSY = 0x08;
static const uint8_t PMC_FLAG_ERROR = 0x10;

PanelControlSource decodeSelectedSourceFromMFlags(uint8_t flags) {
  if (flags & 0x04) return PANEL_SOURCE_PC;
  if (flags & 0x08) return PANEL_SOURCE_AUTO;
  return PANEL_SOURCE_LOCAL;
}

uint8_t buildPmcFlags() {
  uint8_t flags = 0;
  if (g_estopActive || g_remoteEstop) flags |= PMC_FLAG_ESTOP;
  if (g_positionTrusted) flags |= PMC_FLAG_HOMED;
  if (g_ena == ENA_LEVEL_ENABLE_CFG) flags |= PMC_FLAG_DRIVERS_ENABLED;
  if (isRobotBusy()) flags |= PMC_FLAG_BUSY;
  if (g_hasError) flags |= PMC_FLAG_ERROR;
  return flags;
}

uint8_t mapActiveSourceToPanel() {
  switch (g_controlSource) {
    case CONTROL_SOURCE_ESP:  return PANEL_SOURCE_LOCAL;
    case CONTROL_SOURCE_GUI:  return PANEL_SOURCE_PC;
    case CONTROL_SOURCE_AUTO: return PANEL_SOURCE_AUTO;
    case CONTROL_SOURCE_NONE:
    default:
      return (uint8_t)g_panelSelectedSource;
  }
}

uint8_t buildRunState() {
  if (g_estopActive || g_remoteEstop) return PMC_RUN_ERROR;
  if (g_calibrating) return PMC_RUN_HOMING;
  if (g_sequenceActive) return PMC_RUN_AUTO_RUNNING;
  if (isRobotBusy()) return PMC_RUN_BUSY;
  if (g_hasError) return PMC_RUN_ERROR;
  return PMC_RUN_READY;
}
}  // namespace

void espInterfaceInit() {
  Wire.begin();
  Wire.setClock(I2C_FREQ);
  Wire.setTimeout(50);
}

bool readCmdCheckedFrom(uint8_t addr, uint8_t cmd, uint8_t* payload, size_t payloadLen) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) return false;

  const size_t total = payloadLen + 1;
  uint8_t buf[32];
  if (total > sizeof(buf)) return false;

  size_t got = Wire.requestFrom((int)addr, (int)total, (int)true);
  if (got != total) return false;

  for (size_t i = 0; i < total; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }

  if (buf[0] != cmd) return false;

  for (size_t i = 0; i < payloadLen; i++) {
    payload[i] = buf[i + 1];
  }

  return true;
}

bool readCmdChecked(uint8_t cmd, uint8_t* payload, size_t payloadLen) {
  return readCmdCheckedFrom(ESP32_ADDR, cmd, payload, payloadLen);
}

bool readCmdCheckedFromRetry(uint8_t addr, uint8_t cmd, uint8_t* payload, size_t payloadLen) {
  if (readCmdCheckedFrom(addr, cmd, payload, payloadLen)) return true;
  delay(2);
  return readCmdCheckedFrom(addr, cmd, payload, payloadLen);
}

bool readCmdCheckedRetry(uint8_t cmd, uint8_t* payload, size_t payloadLen) {
  if (readCmdChecked(cmd, payload, payloadLen)) return true;
  delay(2);
  return readCmdChecked(cmd, payload, payloadLen);
}

bool pollRemoteEstopDuringBlockingMotion() {
  if (g_estopActive || g_remoteEstop) return true;

  static uint8_t activeSamples = 0;

  // Limita el trafico I2C, pero permite detectar el ESTOP aun cuando una
  // calibracion o movimiento sincronizado mantiene ocupado el loop principal.
  static uint32_t lastPollMs = 0;
  const uint32_t now = millis();
  if (now - lastPollMs < 10) return false;
  lastPollMs = now;

  uint8_t p[4];
  if (!readCmdCheckedRetry('M', p, 4)) {
    activeSamples = 0;
    return false;
  }

  const bool panelEstop = (p[3] & 0x01) != 0;
  if (!panelEstop) {
    activeSamples = 0;
    return false;
  }

  if (activeSamples < ESTOP_CONFIRM_SAMPLES) activeSamples++;
  if (activeSamples < ESTOP_CONFIRM_SAMPLES) return false;
  activeSamples = 0;

  g_remoteEstop = true;
  activateEmergencyStop();
  Serial.println("ESTOP detectado durante movimiento bloqueante.");
  return true;
}

static int16_t readI16LE(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool readHuskyStateI2C(PadState& st) {

  // El comando N devuelve 31 bytes de datos (32 con el eco):
  //
  // flags       1 byte
  // count       1
  // x           2
  // y           2
  // w           2
  // h           2
  // xMm10       2
  // yMm10       2
  // name        17 (hasta 16 caracteres + terminador NUL)
  //
  // Total payload = 31 bytes

  uint8_t p[31];

  if (!readCmdCheckedFromRetry(
      HUSKY_ESP32_ADDR,
      'N',
      p,
      sizeof(p)
  )) {
    return false;
  }

  st.huskyFlags = p[0];

  st.huskyCount = p[1];

  st.huskyX = readI16LE(&p[2]);
  st.huskyY = readI16LE(&p[4]);

  st.huskyW = readI16LE(&p[6]);
  st.huskyH = readI16LE(&p[8]);

  st.huskyXmm10 =
      readI16LE(&p[10]);

  st.huskyYmm10 =
      readI16LE(&p[12]);

  // El receptor comprueba el terminador antes de usar el nombre.
  if ((st.huskyFlags & 0x40) && !memchr(&p[14], 0, sizeof(st.huskyName))) {
    return false;
  }
  memcpy(st.huskyName, &p[14], sizeof(st.huskyName));
  st.huskyName[sizeof(st.huskyName) - 1] = 0;
  if (!(st.huskyFlags & 0x40)) st.huskyName[0] = 0;

  st.huskyDetected =
      (st.huskyFlags & 0x04) != 0;

  st.huskyMapped =
      (st.huskyFlags & 0x20) != 0;

  return true;
}

bool readPadStateI2C(PadState& st) {
  uint8_t mFlags = 0;
  {
    uint8_t p[4];
    if (!readCmdCheckedRetry('M', p, 4)) return false;
    st.menuState  = p[0];
    st.manualMode = p[1];
    st.calibrationMode = (st.menuState == 1) ? p[1] : 0;
    st.misc       = p[2];
    mFlags        = p[3];
    st.estop      = (mFlags & 0x01) != 0;
    st.selectedSource = (uint8_t)decodeSelectedSourceFromMFlags(mFlags);
  }

  {
    uint8_t p[4];
    if (readCmdCheckedRetry('P', p, 4)) {
      st.selectedSource = p[0];
      st.pmcRunState = p[1];
      st.pmcFlags = p[2];
      st.pmcActiveSource = p[3];
      st.pmcStatusValid = true;
    } else {
      st.pmcStatusValid = (mFlags & 0x10) != 0;
      st.pmcRunState = 0;
      st.pmcFlags = 0;
      st.pmcActiveSource = st.selectedSource;
    }
  }

  if (st.menuState == 2 || st.menuState == 6) {
    {
      uint8_t p[2];
      if (readCmdCheckedRetry('B', p, 2)) {
        st.buttons = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
      }
    }
    {
      uint8_t p[1];
      if (readCmdCheckedRetry('C', p, 1)) st.dpad = p[0];
    }
    {
      uint8_t p[4];
      if (readCmdCheckedRetry('J', p, 4)) {
        st.lx = (int8_t)p[0];
        st.ly = (int8_t)p[1];
        st.rx = (int8_t)p[2];
        st.ry = (int8_t)p[3];
      }
    }
    {
      uint8_t p[2];
      if (readCmdCheckedRetry('T', p, 2)) {
        st.lt = p[0];
        st.rt = p[1];
      }
    }
  }

  // Leer vision en Auto y Teach.
  // Si falla esta lectura, no tiramos abajo todo el pad.
  if (st.menuState == 3 || st.menuState == 6) {
    if (!readHuskyStateI2C(st)) {
      st.huskyFlags = 0x10;
      st.huskyDetected = false;
      st.huskyName[0] = 0;
      st.huskyCount = 0;
      st.huskyX = 0;
      st.huskyY = 0;
      st.huskyW = 0;
      st.huskyH = 0;
      st.huskyXmm10 = 0;
      st.huskyYmm10 = 0;
      st.huskyMapped = false;
    }
  }

  g_panelSelectedSource = (PanelControlSource)st.selectedSource;
  return true;
}

bool sendPmcStatusToEsp(uint8_t runState, uint8_t pmcFlags, uint8_t activeSource) {
  Wire.beginTransmission(ESP32_ADDR);
  Wire.write('S');
  Wire.write(runState);
  Wire.write(pmcFlags);
  Wire.write(activeSource);
  uint8_t err = Wire.endTransmission(true);
  return err == 0;
}

bool sendGripperAngleI2C(uint8_t angle) {
  if (angle < GRIPPER_OPEN_ANGLE) angle = GRIPPER_OPEN_ANGLE;
  if (angle > GRIPPER_CLOSED_ANGLE) angle = GRIPPER_CLOSED_ANGLE;
  Wire.beginTransmission(ESP32_ADDR);
  Wire.write('G');
  Wire.write(angle);
  return Wire.endTransmission(true) == 0;
}

bool sendRotationAngleI2C(uint8_t angle) {
  if (angle > 180) return false;
  Wire.beginTransmission(ESP32_ADDR);
  Wire.write('R');
  Wire.write(angle);
  return Wire.endTransmission(true) == 0;
}

bool openGripperI2C() {
  return sendGripperAngleI2C(GRIPPER_OPEN_ANGLE);
}

bool closeGripperI2C() {
  return sendGripperAngleI2C(GRIPPER_CLOSED_ANGLE);
}

void publishPmcStatusToEspIfNeeded(bool force) {
  static uint32_t lastTxMs = 0;
  if (!force && (millis() - lastTxMs < 100)) return;
  lastTxMs = millis();

  (void)sendPmcStatusToEsp(buildRunState(), buildPmcFlags(), mapActiveSourceToPanel());
}

bool sendCmdTo(uint8_t addr, uint8_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  uint8_t err = Wire.endTransmission(true);
  return err == 0;
}

bool startHuskyTagCalibrationI2C() {
  return sendCmdTo(HUSKY_ESP32_ADDR, 'C');
}

static int16_t readI16LE2(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool readHuskyCalibrationStateI2C(HuskyCalibState& out) {
  uint8_t p[9];

  if (!readCmdCheckedFromRetry(HUSKY_ESP32_ADDR, 'K', p, 9)) {
    return false;
  }

  out.flags = p[0];
  out.camXMin = readI16LE2(&p[1]);
  out.camXMax = readI16LE2(&p[3]);
  out.camYMin = readI16LE2(&p[5]);
  out.camYMax = readI16LE2(&p[7]);

  return true;
}
