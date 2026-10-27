#pragma once

#include "types.h"

ManualMode getManualMode(const PadState& st);
void handleMenuCalibration(const PadState& st);
void handleMenuManual(const PadState& st);
void handleMenuAuto(const PadState& st);
void handleMenuTeach(const PadState& st);
void dispatchPadState(const PadState& st);
void dispatchPendingGuiCommands();
