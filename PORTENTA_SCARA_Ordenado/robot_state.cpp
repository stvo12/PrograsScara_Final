#include "robot_state.h"
#include "scara_core_bridge.h"
#include "project_config.h"
#include "motion_service.h"

ScaraPose::PoseState g_poseState = {};
MotorMap::MotorMapParams g_motorParams = MotorMap::defaultParams();
MotorMap::MotorLimits g_motorLimits = MotorMap::defaultLimits();
Safety::SafetyConfig g_safetyCfg = Safety::defaultConfig();

bool g_cartesianControlEnabled = false;
bool g_calibrationDone = false;
bool g_calDone = false;
bool g_calibrating = false;
bool g_visualCalibrationDone = false;
bool g_positionTrusted = false;
bool g_armPositionTrusted = false;
bool g_estopActive = false;
bool g_homeTransitionActive = false;
bool g_remoteEstop = false;
bool g_guiActive = false;
bool g_sequenceActive = false;
bool g_busy = false;
bool g_hasError = false;

unsigned long g_lastControlMs = 0;
uint8_t g_lastManualMode = 0;
uint32_t g_lastPrintMs = 0;
ControlSource g_controlSource = CONTROL_SOURCE_NONE;
PanelControlSource g_panelSelectedSource = PANEL_SOURCE_LOCAL;
const char* g_lastErrorText = "OK";

long posSteps_M1 = 0;
long posSteps_M2 = 0;
long posSteps_M3 = 0;

long M1_minSteps = 0;
long M1_maxSteps = 0;
long M2_minSteps = 0;
long M2_maxSteps = 0;
long M1_homeSteps = 0;
long M2_homeSteps = 0;
long M3_homeSteps = 0;

int di_last[7] = {0, 0, 0, 0, 0, 0, 0};
bool di_menu4_initialized = false;

void resetRobotStateForStop() {
  g_lastManualMode = MANUAL_MODE_NONE;
  g_stop = true;
  g_cartesianControlEnabled = false;
  g_controlSource = CONTROL_SOURCE_NONE;
  g_guiActive = false;
  g_busy = false;
}

void markControlSource(ControlSource src) {
  g_controlSource = src;
}

void clearRobotError() {
  g_hasError = false;
  g_lastErrorText = "OK";
}

void setRobotError(const char* text) {
  g_hasError = true;
  g_lastErrorText = (text != nullptr) ? text : "ERROR";
}

bool isRobotBusy() {
  return g_busy || g_calibrating || g_sequenceActive || g_homeTransitionActive;
}

void activateEmergencyStop() {
  g_estopActive = true;
  g_stop = true;
  g_positionTrusted = false;
  g_armPositionTrusted = false;
  g_calibrationDone = false;
  g_calDone = false;
  g_cartesianControlEnabled = false;
  g_lastManualMode = MANUAL_MODE_NONE;
  setRobotError("ESTOP_ACTIVE");
  enaWriteOnce(ENA_LEVEL_DISABLE_CFG);
}

void releaseEmergencyStop() {
  g_estopActive = false;
  g_stop = true;
  setRobotError("POSITION_UNTRUSTED");
}
