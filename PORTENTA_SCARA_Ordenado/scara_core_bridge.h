#pragma once

#include <Arduino.h>
#include <Arduino_PortentaMachineControl.h>

// Este header expone solo los símbolos del core original que sí se usan
// fuera del .ino, evitando incluir ScaraCalibrationCore.h en todos los .cpp.

extern volatile bool g_stop;
extern PinStatus g_ena;

extern unsigned int g_PULSE12_US;
extern unsigned int g_PULSE3_US;

long moveFixedSingle(uint8_t stepCh, uint8_t dirCh, PinStatus dirLevel, long steps, unsigned int pulseUs);
