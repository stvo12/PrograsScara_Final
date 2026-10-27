#pragma once

#include <Arduino.h>
#include "types.h"

void sequenceRunnerInit();
bool startPickSequence(float pickX, float pickY, float pickZ, float safeZ,
                       uint8_t graspAngle);
bool startPickPlaceSequence(const PickPlaceParams& params);
bool startPrepositionedPickPlaceSequence(const PickPlaceParams& params,
                                        bool closeAlreadySent = false,
                                        uint32_t closeSentMs = 0);
void updateSequenceRunner();
bool isSequenceRunning();
