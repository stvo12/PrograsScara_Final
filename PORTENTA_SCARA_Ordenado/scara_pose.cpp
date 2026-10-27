#include "scara_pose.h"
#include "project_config.h"
#include <math.h>

namespace ScaraPose {

PoseConfig defaultConfig() {
  PoseConfig cfg;
  cfg.deadzone = 0.08f;
  cfg.maxVxy   = 80.0f;   // mm/s
  cfg.maxVz    = 50.0f;   // mm/s

  // Mantener el joystick cartesiano dentro del mismo recorrido fisico de M3.
  cfg.zMin = M3_MIN_MM;
  cfg.zMax = M3_MAX_MM;
  return cfg;
}

void init(PoseState& state, const Pose& initialPose) {
  state.desired = initialPose;
  state.initialized = true;
}

void setDesiredPose(PoseState& state, const Pose& pose) {
  state.desired = pose;
  state.initialized = true;
}

Pose getDesiredPose(const PoseState& state) {
  return state.desired;
}

bool isInitialized(const PoseState& state) {
  return state.initialized;
}

float normalizeAxis(int value, int maxAbs) {
  if (maxAbs <= 0) return 0.0f;

  float u = (float)value / (float)maxAbs;
  if (u > 1.0f)  u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  return u;
}

float applyDeadzone(float u, float deadzone) {
  if (fabsf(u) < deadzone) {
    return 0.0f;
  }

  if (u > 0.0f) {
    return (u - deadzone) / (1.0f - deadzone);
  } else {
    return (u + deadzone) / (1.0f - deadzone);
  }
}

void clampPose(Pose& pose, const PoseConfig& cfg) {
  if (pose.z < cfg.zMin) pose.z = cfg.zMin;
  if (pose.z > cfg.zMax) pose.z = cfg.zMax;
}

void updateFromJoystick(PoseState& state,
                        int lx, int ly, int ry,
                        float dt,
                        const PoseConfig& cfg) {
  if (!state.initialized || dt <= 0.0f) {
    return;
  }

  float ux = normalizeAxis(lx);
  float uy = normalizeAxis(ly);
  float uz = normalizeAxis(ry);

  ux = applyDeadzone(ux, cfg.deadzone);
  uy = applyDeadzone(uy, cfg.deadzone);
  uz = applyDeadzone(uz, cfg.deadzone);

  float vx = cfg.maxVxy * ux;  // mm/s
  float vy = cfg.maxVxy * uy;  // mm/s
  float vz = cfg.maxVz  * uz;  // mm/s

  state.desired.x += vx * dt;
  state.desired.y += vy * dt;
  state.desired.z += vz * dt;

  clampPose(state.desired, cfg);
}

}  // namespace ScaraPose
