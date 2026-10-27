#pragma once

#include <Arduino.h>
#include "types.h"

void guiProtocolInit();
void updateGuiProtocol();
bool guiHasPendingCommand();
GuiCommand guiTakePendingCommand();
