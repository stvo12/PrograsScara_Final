#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>
#include "kinematics.h"
#include "motor_map.h"

namespace Safety {

using Kinematics::Pose;
using Kinematics::Joints;
using Kinematics::RobotParams;
using MotorMap::MotorTargets;
using MotorMap::MotorLimits;

// Resultado de validación
enum class SafetyCode : uint8_t {
  OK = 0,
  INVALID_POSE,
  OUTSIDE_WORKSPACE_XY,
  Z_OUT_OF_RANGE,
  IK_FAILED,
  INVALID_JOINTS,
  MOTOR_LIMIT_EXCEEDED,
  NOT_CALIBRATED
};

// Configuración de seguridad
struct SafetyConfig {
  float zMin;   // mm
  float zMax;   // mm
};

// Configuración por defecto
SafetyConfig defaultConfig();

// Validaciones unitarias
bool isPoseFinite(const Pose& p);
bool isJointsFinite(const Joints& q);
bool isPoseWithinWorkspace(const Pose& p,
                           const SafetyConfig& cfg = defaultConfig(),
                           const RobotParams& rp = Kinematics::defaultParams());

bool isMotorCommandSafe(const MotorTargets& m,
                        const MotorLimits& lim);

// Flujo completo de validación
SafetyCode validatePose(const Pose& p,
                        const SafetyConfig& cfg = defaultConfig(),
                        const RobotParams& rp = Kinematics::defaultParams());

SafetyCode validateCommand(const Pose& p,
                           const Joints& q,
                           const MotorTargets& m,
                           bool calibrated,
                           const MotorLimits& lim,
                           const SafetyConfig& cfg = defaultConfig(),
                           const RobotParams& rp = Kinematics::defaultParams());

// Texto útil para depuración
const char* toString(SafetyCode code);

}  // namespace Safety

#endif