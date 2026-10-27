#ifndef MOTOR_MAP_H
#define MOTOR_MAP_H

#include <Arduino.h>
#include "kinematics.h"

namespace MotorMap {

using Kinematics::Joints;

struct MotorTargets {
  long m1;
  long m2;
  long m3;
};

struct MotorMapParams {
  float m1_steps_per_rad;
  float m2_steps_per_rad;
  float m3_steps_per_mm;

  long m1_zero;
  long m2_zero;
  long m3_zero;

  int m1_sign;
  int m2_sign;
  int m3_sign;

  // Home articular real asociado al home mecánico
  float q1_home_rad;
  float q2_home_rad;
  float q3_home_mm;
};

struct MotorLimits {
  long m1_min;
  long m1_max;
  long m2_min;
  long m2_max;
  long m3_min;
  long m3_max;
};

MotorMapParams defaultParams();
MotorLimits defaultLimits();

MotorLimits makeLimits(long m1_min, long m1_max,
                       long m2_min, long m2_max,
                       long m3_min, long m3_max);

void setZeros(MotorMapParams& p,
              long m1_zero, long m2_zero, long m3_zero);

void setHomeJoints(MotorMapParams& p,
                   float q1_home_rad,
                   float q2_home_rad,
                   float q3_home_mm);

bool jointsToMotor(const Joints& q,
                   MotorTargets& out,
                   const MotorMapParams& p = defaultParams());

bool motorToJoints(const MotorTargets& m,
                   Joints& qOut,
                   const MotorMapParams& p = defaultParams());

bool withinMotorLimits(const MotorTargets& m,
                       const MotorLimits& lim);

}  // namespace MotorMap

#endif