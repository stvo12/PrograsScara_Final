#pragma once

#include <Arduino.h>
#include "types.h"

void printPadStateIfNeeded(const PadState& st, bool ok);
void resetTestDiPrinter();
void handleMenuTestDI(const PadState& st);
void printCompactRobotStatus(Stream& out);
