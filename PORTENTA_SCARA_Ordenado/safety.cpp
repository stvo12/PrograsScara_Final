#include "safety.h"
#include "project_config.h"
#include <math.h>

namespace Safety {

SafetyConfig defaultConfig() {
  SafetyConfig cfg;
  // El espacio cartesiano de Z usa los mismos limites fisicos configurables
  // que la conversion de distancia a pasos de M3.
  cfg.zMin = M3_MIN_MM;
  cfg.zMax = M3_MAX_MM;
  return cfg;
}

bool isPoseFinite(const Pose& p) {
  return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

bool isJointsFinite(const Joints& q) {
  return isfinite(q.q1) && isfinite(q.q2) && isfinite(q.q3);
}

bool isPoseWithinWorkspace(const Pose& p,
                           const SafetyConfig& cfg,
                           const RobotParams& rp) {
  if (!isPoseFinite(p)) {
    return false;
  }

  if (!Kinematics::withinWorkspaceXY(p.x, p.y, rp)) {
    return false;
  }

  if (p.z < cfg.zMin || p.z > cfg.zMax) {
    return false;
  }

  return true;
}

bool isMotorCommandSafe(const MotorTargets& m,
                        const MotorLimits& lim) {
  return MotorMap::withinMotorLimits(m, lim);
}

SafetyCode validatePose(const Pose& p,
                        const SafetyConfig& cfg,
                        const RobotParams& rp) {
  if (!isPoseFinite(p)) {
    return SafetyCode::INVALID_POSE;
  }

  if (!Kinematics::withinWorkspaceXY(p.x, p.y, rp)) {
    return SafetyCode::OUTSIDE_WORKSPACE_XY;
  }

  if (p.z < cfg.zMin || p.z > cfg.zMax) {
    return SafetyCode::Z_OUT_OF_RANGE;
  }

  return SafetyCode::OK;
}

SafetyCode validateCommand(const Pose& p,
                           const Joints& q,
                           const MotorTargets& m,
                           bool calibrated,
                           const MotorLimits& lim,
                           const SafetyConfig& cfg,
                           const RobotParams& rp) {
  if (!calibrated) {
    return SafetyCode::NOT_CALIBRATED;
  }

  SafetyCode poseCheck = validatePose(p, cfg, rp);
  if (poseCheck != SafetyCode::OK) {
    return poseCheck;
  }

  if (!isJointsFinite(q)) {
    return SafetyCode::INVALID_JOINTS;
  }

  if (!isMotorCommandSafe(m, lim)) {
    return SafetyCode::MOTOR_LIMIT_EXCEEDED;
  }

  return SafetyCode::OK;
}

const char* toString(SafetyCode code) {
  switch (code) {
    case SafetyCode::OK:
      return "OK";
    case SafetyCode::INVALID_POSE:
      return "INVALID_POSE";
    case SafetyCode::OUTSIDE_WORKSPACE_XY:
      return "OUTSIDE_WORKSPACE_XY";
    case SafetyCode::Z_OUT_OF_RANGE:
      return "Z_OUT_OF_RANGE";
    case SafetyCode::IK_FAILED:
      return "IK_FAILED";
    case SafetyCode::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case SafetyCode::MOTOR_LIMIT_EXCEEDED:
      return "MOTOR_LIMIT_EXCEEDED";
    case SafetyCode::NOT_CALIBRATED:
      return "NOT_CALIBRATED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace Safety
