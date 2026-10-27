#pragma once

#include <Arduino.h>
#include "types.h"
#include "scara_pose.h"
#include "motor_map.h"
#include "safety.h"

extern ScaraPose::PoseState g_poseState;
extern MotorMap::MotorMapParams g_motorParams;
extern MotorMap::MotorLimits g_motorLimits;
extern Safety::SafetyConfig g_safetyCfg;

extern bool g_cartesianControlEnabled;
extern bool g_calibrationDone;
extern bool g_calDone;
extern bool g_calibrating;
extern bool g_visualCalibrationDone;
extern bool g_positionTrusted;
// Referencia independiente de M1/M2 para recuperar solo Z sin asumir XY valido.
extern bool g_armPositionTrusted;
extern bool g_estopActive;
extern bool g_homeTransitionActive;
extern bool g_remoteEstop;
extern bool g_guiActive;
extern bool g_sequenceActive;
extern bool g_busy;
extern bool g_hasError;

extern unsigned long g_lastControlMs;
extern uint8_t g_lastManualMode;
extern uint32_t g_lastPrintMs;
extern ControlSource g_controlSource;
extern PanelControlSource g_panelSelectedSource;
extern const char* g_lastErrorText;

extern long posSteps_M1;
extern long posSteps_M2;
extern long posSteps_M3;

extern long M1_minSteps;
extern long M1_maxSteps;
extern long M2_minSteps;
extern long M2_maxSteps;
extern long M1_homeSteps;
extern long M2_homeSteps;
extern long M3_homeSteps;

extern int di_last[7];
extern bool di_menu4_initialized;

void resetRobotStateForStop();
void markControlSource(ControlSource src);
void clearRobotError();
void setRobotError(const char* text);
bool isRobotBusy();
void activateEmergencyStop();
void releaseEmergencyStop();
