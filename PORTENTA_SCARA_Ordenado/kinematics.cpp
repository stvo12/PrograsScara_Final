#include "kinematics.h"
#include "project_config.h"
#include <math.h>

namespace Kinematics {

// ============================================================
// Parámetros por defecto del SCARA
// ============================================================
RobotParams defaultParams() {
  RobotParams p;
  p.h_base = 176.0f;
  p.d1     = 26.5f;
  p.L1     = SCARA_L1_MM;
  p.L2     = SCARA_L2_MM;
  p.dtool  = 85.0f;

  // z = h_base + d1 - q3 - dtool. Por eso los limites de q3
  // se obtienen invirtiendo los limites fisicos configurables de M3.
  p.q3_min = p.h_base + p.d1 - p.dtool - M3_MAX_MM;
  p.q3_max = p.h_base + p.d1 - p.dtool - M3_MIN_MM;
  return p;
}

// ============================================================
// Utilidades
// ============================================================
float wrapAngle(float a) {
  return atan2f(sinf(a), cosf(a));
}

bool isFinitePose(const Pose& p) {
  return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

bool isFiniteJoints(const Joints& q) {
  return isfinite(q.q1) && isfinite(q.q2) && isfinite(q.q3);
}

bool withinQ3Limits(float q3, const RobotParams& p) {
  return (q3 >= p.q3_min && q3 <= p.q3_max);
}

bool withinWorkspaceXY(float x, float y, const RobotParams& p) {
  const float r = sqrtf(x * x + y * y);
  const float rMin = fabsf(p.L1 - p.L2);
  const float rMax = p.L1 + p.L2;
  return (r >= rMin && r <= rMax);
}

// ============================================================
// Cinemática directa
// Modelo:
// x = L1*cos(q1) + L2*cos(q1+q2)
// y = L1*sin(q1) + L2*sin(q1+q2)
// z = h_base + d1 - q3 - dtool
// ============================================================
Pose robotFK(const Joints& q, const RobotParams& p) {
  Pose out;

  const float q12 = q.q1 + q.q2;

  out.x = p.L1 * cosf(q.q1) + p.L2 * cosf(q12);
  out.y = p.L1 * sinf(q.q1) + p.L2 * sinf(q12);
  out.z = p.h_base + p.d1 - q.q3 - p.dtool;

  return out;
}

// ============================================================
// Cinemática inversa
// ============================================================
bool robotIK(const Pose& pose, Joints& qOut, bool elbowUp, const RobotParams& p) {
  if (!isFinitePose(pose)) {
    return false;
  }

  const float x = pose.x;
  const float y = pose.y;
  const float z = pose.z;

  const float r2 = x * x + y * y;
  const float denom = 2.0f * p.L1 * p.L2;

  if (denom == 0.0f) {
    return false;
  }

  float c2 = (r2 - p.L1 * p.L1 - p.L2 * p.L2) / denom;

  // Protección numérica
  if (c2 > 1.0f + 1e-5f || c2 < -1.0f - 1e-5f) {
    return false;
  }
  if (c2 > 1.0f) c2 = 1.0f;
  if (c2 < -1.0f) c2 = -1.0f;

  float s2_sq = 1.0f - c2 * c2;
  if (s2_sq < 0.0f) s2_sq = 0.0f;

  float s2 = sqrtf(s2_sq);
  if (!elbowUp) {
    s2 = -s2;
  }

  const float q2 = atan2f(s2, c2);
  const float q1 = atan2f(y, x) - atan2f(p.L2 * s2, p.L1 + p.L2 * c2);

  // z = h_base + d1 - q3 - dtool
  const float q3 = p.h_base + p.d1 - p.dtool - z;

  if (!withinQ3Limits(q3, p)) {
    return false;
  }

  qOut.q1 = wrapAngle(q1);
  qOut.q2 = wrapAngle(q2);
  qOut.q3 = q3;

  return isFiniteJoints(qOut);
}

}  // namespace Kinematics
