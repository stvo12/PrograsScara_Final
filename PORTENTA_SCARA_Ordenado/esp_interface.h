#pragma once

#include <Arduino.h>
#include "types.h"

struct HuskyCalibState {
  uint8_t flags;
  int16_t camXMin;
  int16_t camXMax;
  int16_t camYMin;
  int16_t camYMax;
};

bool startHuskyTagCalibrationI2C();
bool readHuskyCalibrationStateI2C(HuskyCalibState& out);

void espInterfaceInit();
bool readCmdChecked(uint8_t cmd, uint8_t* payload, size_t payloadLen);
bool readCmdCheckedRetry(uint8_t cmd, uint8_t* payload, size_t payloadLen);
bool readPadStateI2C(PadState& st);
bool pollRemoteEstopDuringBlockingMotion();
bool sendPmcStatusToEsp(uint8_t runState, uint8_t pmcFlags, uint8_t activeSource);
bool sendGripperAngleI2C(uint8_t angle);
bool sendRotationAngleI2C(uint8_t angle);
bool openGripperI2C();
bool closeGripperI2C();
void publishPmcStatusToEspIfNeeded(bool force = false);
bool readHuskyStateI2C(PadState& st);
