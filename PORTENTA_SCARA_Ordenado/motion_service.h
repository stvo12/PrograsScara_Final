#pragma once

#include <Arduino.h>
#include "types.h"
#include "kinematics.h"
#include "motor_map.h"

void motionServiceInitHardware();
void enaWriteOnce(PinStatus level);
bool endstopActive(int di_pin);

long stepMotor1(bool positive, long steps, uint16_t pulse_us);
long stepMotor2(bool positive, long steps, uint16_t pulse_us);
long stepMotor3(bool positive, long steps, uint16_t pulse_us);

bool moveM1ToSteps(long target);
bool moveM2ToSteps(long target);
bool moveM3ToSteps(long target);
using MotionProgressHook = bool (*)();
uint16_t selectAutoCoordinatedPulseUs(const MotorMap::MotorTargets& target,
                                    uint16_t fallbackPulseUs);
bool moveCartesianTargets(const MotorMap::MotorTargets& mCmd,
                          MotionProgressHook progressHook = nullptr);

void applyCalibrationToCartesianControl();
void initCartesianPoseFromCurrentSteps();
bool solveIKNearestCurrent(const Kinematics::Pose& poseCmd,
                           Kinematics::Joints& qBest,
                           MotorMap::MotorTargets& mBest);
void processCartesianJoystickControl(int lx, int ly, int ry);

long axisToJogDelta(int8_t v, long maxDelta);
bool moveToPoseXYZ(float x, float y, float z,
                   MotionProgressHook progressHook = nullptr);
bool moveToJoints(float q1, float q2, float q3);
void handleManualJoint(const PadState& st);
void handleManualCartesian(const PadState& st);

bool moveMotorTargetsSync(const MotorMap::MotorTargets& target, uint16_t pulse_us,
                         MotionProgressHook progressHook = nullptr);
