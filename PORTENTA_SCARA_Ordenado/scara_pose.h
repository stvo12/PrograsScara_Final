#ifndef SCARA_POSE_H
#define SCARA_POSE_H

#include <Arduino.h>
#include "kinematics.h"

namespace ScaraPose {

using Kinematics::Pose;

struct PoseConfig {
  float deadzone;   // 0.0 a 1.0
  float maxVxy;     // mm/s
  float maxVz;      // mm/s

  float zMin;       // mm
  float zMax;       // mm
};

struct PoseState {
  Pose desired;
  bool initialized;
};

PoseConfig defaultConfig();

void init(PoseState& state, const Pose& initialPose);
void setDesiredPose(PoseState& state, const Pose& pose);
Pose getDesiredPose(const PoseState& state);
bool isInitialized(const PoseState& state);

// lx, ly, ry: ejes del joystick
// lx -> x
// ly -> y
// ry -> z
void updateFromJoystick(PoseState& state,
                        int lx, int ly, int ry,
                        float dt,
                        const PoseConfig& cfg = defaultConfig());

float normalizeAxis(int value, int maxAbs = 127);
float applyDeadzone(float u, float deadzone);
void clampPose(Pose& pose, const PoseConfig& cfg);

}  // namespace ScaraPose

#endif