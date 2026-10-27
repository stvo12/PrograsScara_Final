#pragma once

#include <Arduino.h>

bool seekEndstopM1(int di_pin, bool movePositive);
bool seekEndstopM2(int di_pin, bool movePositive);
bool runCalibrationSequence();
bool runM3CalibrationSequence();
bool moveRobotToCalibratedHome();
