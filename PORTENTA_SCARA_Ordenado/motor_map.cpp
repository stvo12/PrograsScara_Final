#include "motor_map.h"
#include "project_config.h"
#include <math.h>

namespace MotorMap {

static constexpr float STEPS_PER_REV = 1600.0f;
static constexpr float R1 = 5.8f;

MotorMapParams defaultParams() {
  MotorMapParams p;

  p.m1_steps_per_rad = (STEPS_PER_REV * R1) / (2.0f * PI);
  const float m2Ratio = M2_MOTOR_PULLEY_TEETH > 0
      ? (float)M2_JOINT_PULLEY_TEETH / (float)M2_MOTOR_PULLEY_TEETH : 0.0f;
  p.m2_steps_per_rad = (STEPS_PER_REV * m2Ratio) / (2.0f * PI);
  p.m3_steps_per_mm = m3ConfiguredStepsPerMm();

  p.m1_zero = 0;
  p.m2_zero = 0;
  p.m3_zero = 0;

  p.m1_sign = 1;
  p.m2_sign = 1;
  // Desde el final de carrera de pick (M3=0), aumentar steps eleva Z.
  // En el modelo SCARA, elevar Z disminuye q3.
  p.m3_sign = -1;

  // Valores iniciales razonables. Ajustables luego.
  p.q1_home_rad = 0.0f;
  p.q2_home_rad = -PI / 2.0f;
  // q3 correspondiente a Z=0: h_base + d1 - dtool = 117.5 mm.
  // Esta es la referencia física M3=0, no el HOME retirado de 1000 steps.
  p.q3_home_mm  = 117.5f;

  return p;
}

MotorLimits defaultLimits() {
  MotorLimits lim;
  lim.m1_min = -100000;
  lim.m1_max =  100000;
  lim.m2_min = -100000;
  lim.m2_max =  100000;
  lim.m3_min = -100000;
  lim.m3_max =  100000;
  return lim;
}

MotorLimits makeLimits(long m1_min, long m1_max,
                       long m2_min, long m2_max,
                       long m3_min, long m3_max) {
  MotorLimits lim;
  lim.m1_min = m1_min;
  lim.m1_max = m1_max;
  lim.m2_min = m2_min;
  lim.m2_max = m2_max;
  lim.m3_min = m3_min;
  lim.m3_max = m3_max;
  return lim;
}

void setZeros(MotorMapParams& p,
              long m1_zero, long m2_zero, long m3_zero) {
  p.m1_zero = m1_zero;
  p.m2_zero = m2_zero;
  p.m3_zero = m3_zero;
}

void setHomeJoints(MotorMapParams& p,
                   float q1_home_rad,
                   float q2_home_rad,
                   float q3_home_mm) {
  p.q1_home_rad = q1_home_rad;
  p.q2_home_rad = q2_home_rad;
  p.q3_home_mm  = q3_home_mm;
}

bool jointsToMotor(const Joints& q,
                   MotorTargets& out,
                   const MotorMapParams& p) {
  if (!isfinite(q.q1) || !isfinite(q.q2) || !isfinite(q.q3)) {
    return false;
  }

  out.m1 = lroundf((float)p.m1_zero +
                   (float)p.m1_sign * (q.q1 - p.q1_home_rad) * p.m1_steps_per_rad);

  out.m2 = lroundf((float)p.m2_zero +
                   (float)p.m2_sign * (q.q2 - p.q2_home_rad) * p.m2_steps_per_rad);

  out.m3 = lroundf((float)p.m3_zero +
                   (float)p.m3_sign * (q.q3 - p.q3_home_mm) * p.m3_steps_per_mm);

  return true;
}

bool motorToJoints(const MotorTargets& m,
                   Joints& qOut,
                   const MotorMapParams& p) {
  if (p.m1_steps_per_rad == 0.0f ||
      p.m2_steps_per_rad == 0.0f ||
      p.m3_steps_per_mm  == 0.0f) {
    return false;
  }

  qOut.q1 = p.q1_home_rad +
            ((float)(m.m1 - p.m1_zero)) / ((float)p.m1_sign * p.m1_steps_per_rad);

  qOut.q2 = p.q2_home_rad +
            ((float)(m.m2 - p.m2_zero)) / ((float)p.m2_sign * p.m2_steps_per_rad);

  qOut.q3 = p.q3_home_mm +
            ((float)(m.m3 - p.m3_zero)) / ((float)p.m3_sign * p.m3_steps_per_mm);

  return isfinite(qOut.q1) && isfinite(qOut.q2) && isfinite(qOut.q3);
}

bool withinMotorLimits(const MotorTargets& m,
                       const MotorLimits& lim) {
  if (m.m1 < lim.m1_min || m.m1 > lim.m1_max) return false;
  if (m.m2 < lim.m2_min || m.m2 > lim.m2_max) return false;
  if (m.m3 < lim.m3_min || m.m3 > lim.m3_max) return false;
  return true;
}

}  // namespace MotorMap
