#include "motion_service.h"
#include "scara_core_bridge.h"

#include "Arduino_PortentaMachineControl.h"
#include "project_config.h"
#include "robot_state.h"
#include "safety.h"
#include "scara_pose.h"
#include "esp_interface.h"

static constexpr unsigned int DIR_SETUP_US_SYNC = 30;

static bool isOperationalAutoMotion() {
  return g_controlSource == CONTROL_SOURCE_AUTO && !g_calibrating;
}

uint16_t selectAutoCoordinatedPulseUs(const MotorMap::MotorTargets& target,
                                    uint16_t fallbackPulseUs) {
  if (!isOperationalAutoMotion()) return fallbackPulseUs;
  if (target.m3 < posSteps_M3) return AUTO_M3_DESCENT_PULSE_US;
  if (target.m3 > posSteps_M3) return AUTO_M3_ASCENT_PULSE_US;
  return fallbackPulseUs;
}

bool endstopActive(int di_pin) {
  return MachineControl_DigitalInputs.read(di_pin) == ENDSTOP_ACTIVE_LEVEL;
}

// Confirmacion usada durante movimientos operativos. Un unico pulso por ruido
// no debe abortar el pick/place ni invalidar la referencia del robot.
static bool endstopActiveConfirmed(int di_pin) {
  static constexpr uint8_t CONFIRM_SAMPLES = 10;
  static constexpr uint16_t SAMPLE_INTERVAL_US = 1000;

  if (!endstopActive(di_pin)) return false;

  for (uint8_t sample = 1; sample < CONFIRM_SAMPLES; sample++) {
    delayMicroseconds(SAMPLE_INTERVAL_US);
    if (!endstopActive(di_pin)) return false;
  }

  return true;
}

static void syncPoseAfterM3Zero() {
  if (!g_positionTrusted || !g_calibrationDone || g_calibrating) return;
  MotorMap::MotorTargets current;
  current.m1 = posSteps_M1;
  current.m2 = posSteps_M2;
  current.m3 = posSteps_M3;
  Kinematics::Joints joints;
  if (!MotorMap::motorToJoints(current, joints, g_motorParams)) return;
  const Kinematics::Pose pose = Kinematics::robotFK(joints);
  if (!Kinematics::isFinitePose(pose)) return;
  ScaraPose::init(g_poseState, pose);
  g_lastControlMs = millis();
}

// Solo llamar despues de confirmar DI06. Corregir Z no restaura confianza XY.
static void rebaseM3ZeroFromSwitch() {
  const long previousSteps = posSteps_M3;
  posSteps_M3 = M3_MIN_STEPS_PHYS;
  syncPoseAfterM3Zero();
  Serial.print("M3_ZERO_REBASED old_steps="); Serial.print(previousSteps);
  Serial.print(" new_zero_steps="); Serial.print(posSteps_M3);
  Serial.print(" max_steps_unchanged="); Serial.println(M3_MAX_STEPS_PHYS);
}

void enaWriteOnce(PinStatus level) {
  if (g_ena == level) return;
  g_ena = level;
  if (level == ENA_LEVEL_DISABLE_CFG) g_armPositionTrusted = false;
  MachineControl_DigitalOutputs.write(ENA_CH_CFG, g_ena);

  Serial.print("ENA => ");
  Serial.println((g_ena == ENA_LEVEL_ENABLE_CFG) ? "0 (HABILITADO)" : "1 (DESHABILITADO)");
}

void motionServiceInitHardware() {
  MachineControl_DigitalOutputs.begin(true);

  MachineControl_DigitalOutputs.write(STEP1_CH_CFG, LOW);
  MachineControl_DigitalOutputs.write(DIR1_CH_CFG,  LOW);
  MachineControl_DigitalOutputs.write(STEP2_CH_CFG, LOW);
  MachineControl_DigitalOutputs.write(DIR2_CH_CFG,  LOW);
  MachineControl_DigitalOutputs.write(STEP3_CH_CFG, LOW);
  MachineControl_DigitalOutputs.write(DIR3_CH_CFG,  LOW);

  MachineControl_DigitalOutputs.write(ENA_CH_CFG, ENA_LEVEL_DISABLE_CFG);
  g_ena = ENA_LEVEL_DISABLE_CFG;

  Serial.print("ENA INIT => ");
  Serial.println((g_ena == ENA_LEVEL_ENABLE_CFG) ? "0 (HABILITADO)" : "1 (DESHABILITADO)");
}

long stepMotor1(bool positive, long steps, uint16_t pulse_us) {
  if (steps <= 0) return 0;
  PinStatus dir = positive ? dirPosFromNeg(DIR1_LEVEL_NEG_CFG) : DIR1_LEVEL_NEG_CFG;
  moveFixedSingle(STEP1_CH_CFG, DIR1_CH_CFG, dir, steps, pulse_us);
  long d = positive ? steps : -steps;
  posSteps_M1 += d;
  return d;
}

long stepMotor2(bool positive, long steps, uint16_t pulse_us) {
  if (steps <= 0) return 0;
  PinStatus dir = positive ? dirPosFromNeg(DIR2_LEVEL_NEG_CFG) : DIR2_LEVEL_NEG_CFG;
  moveFixedSingle(STEP2_CH_CFG, DIR2_CH_CFG, dir, steps, pulse_us);
  long d = positive ? steps : -steps;
  posSteps_M2 += d;
  return d;
}

long stepMotor3(bool positive, long steps, uint16_t pulse_us) {
  if (steps <= 0) return 0;
  if (!positive && M3_REZERO_ON_ENDSTOP && endstopActiveConfirmed(DI_PINS[6])) {
    rebaseM3ZeroFromSwitch();
    return 0;
  }
  PinStatus dir = positive ? dirPosFromNeg(DIR3_LEVEL_NEG_CFG) : DIR3_LEVEL_NEG_CFG;
  moveFixedSingle(STEP3_CH_CFG, DIR3_CH_CFG, dir, steps, pulse_us);
  long d = positive ? steps : -steps;
  posSteps_M3 += d;
  if (!positive && M3_REZERO_ON_ENDSTOP && endstopActiveConfirmed(DI_PINS[6])) {
    rebaseM3ZeroFromSwitch();
  }
  return d;
}

void applyCalibrationToCartesianControl() {
  g_motorLimits = MotorMap::makeLimits(
    M1_minSteps, M1_maxSteps,
    M2_minSteps, M2_maxSteps,
    M3_MIN_STEPS_PHYS, M3_MAX_STEPS_PHYS
  );

  MotorMap::setZeros(
    g_motorParams,
    M1_homeSteps,
    M2_homeSteps,
    M3_MIN_STEPS_PHYS
  );

  const Kinematics::RobotParams robotParams = Kinematics::defaultParams();
  // La referencia de cero siempre es el final de carrera M3, aunque la
  // altura de pick se configure unos milimetros por encima.
  const float q3AtMin =
      robotParams.h_base + robotParams.d1 -
      robotParams.dtool - M3_MIN_MM;

  MotorMap::setHomeJoints(
    g_motorParams,
    0.0f,
    -PI / 2.0f,
    q3AtMin
  );

  g_calibrationDone = true;
}

bool moveM1ToSteps(long target) {
  if (target < 0) target = 0;
  if (target > M1_maxSteps) target = M1_maxSteps;

  while (posSteps_M1 != target) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;

    long diff = target - posSteps_M1;
    bool positive = (diff > 0);

    if (!positive && endstopActive(DI_PINS[3])) {
      Serial.println("M1 MIN ENDSTOP ACTIVE");
      posSteps_M1 = 0;
      return false;
    }
    if (positive && endstopActive(DI_PINS[2])) {
      Serial.println("M1 MAX ENDSTOP ACTIVE");
      posSteps_M1 = M1_maxSteps;
      return false;
    }

    long chunk = labs(diff);
    if (chunk > HOMING_FAST_CHUNK) chunk = HOMING_FAST_CHUNK;

    stepMotor1(positive, chunk, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }
  return true;
}

bool moveM2ToSteps(long target) {
  if (target < 0) target = 0;
  if (target > M2_maxSteps) target = M2_maxSteps;

  while (posSteps_M2 != target) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;

    long diff = target - posSteps_M2;
    bool positive = (diff > 0);

    if (!positive && endstopActive(DI_PINS[1])) {
      Serial.println("M2 MIN ENDSTOP ACTIVE");
      posSteps_M2 = 0;
      return false;
    }
    if (positive && endstopActive(DI_PINS[0])) {
      Serial.println("M2 MAX ENDSTOP ACTIVE");
      posSteps_M2 = M2_maxSteps;
      return false;
    }

    long chunk = labs(diff);
    if (chunk > HOMING_FAST_CHUNK) chunk = HOMING_FAST_CHUNK;

    stepMotor2(positive, chunk, g_PULSE12_US);
    delay(HOMING_STEP_PAUSE_MS);
  }
  return true;
}

bool moveM3ToSteps(long target) {
  if (target < M3_MIN_STEPS_PHYS) target = M3_MIN_STEPS_PHYS;
  if (target > M3_MAX_STEPS_PHYS) target = M3_MAX_STEPS_PHYS;

  while (posSteps_M3 != target) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;

    long diff = target - posSteps_M3;
    bool positive = (diff > 0);

    if (!positive && endstopActiveConfirmed(DI_PINS[6])) {
      if (!M3_REZERO_ON_ENDSTOP) {
        Serial.println("M3 MIN/HOME ENDSTOP ACTIVE");
        posSteps_M3 = M3_MIN_STEPS_PHYS;
        return false;
      }
      rebaseM3ZeroFromSwitch();
      // Recalcular el sentido desde el nuevo cero; nunca seguir bajando.
      continue;
    }

    long chunk = labs(diff);
    if (chunk > HOMING_FAST_CHUNK) chunk = HOMING_FAST_CHUNK;

    stepMotor3(positive, chunk, g_PULSE3_US);
    delay(HOMING_STEP_PAUSE_MS);
  }
  return true;
}

bool moveCartesianTargets(const MotorMap::MotorTargets& mCmd,
                          MotionProgressHook progressHook) {
  if (g_ena != ENA_LEVEL_ENABLE_CFG) {
    enaWriteOnce(ENA_LEVEL_ENABLE_CFG);
  }

  g_busy = true;

  uint16_t pulseUs = CARTESIAN_SYNC_PULSE_US;

  if (mCmd.m3 < posSteps_M3) {
    pulseUs = selectAutoCoordinatedPulseUs(mCmd, M3_DESCENT_SYNC_PULSE_US);
    Serial.print("M3_DESCENT pulse_us=");
    Serial.println(pulseUs);
  } else if (mCmd.m3 > posSteps_M3) {
    pulseUs = selectAutoCoordinatedPulseUs(mCmd, M3_ASCENT_SYNC_PULSE_US);
    Serial.print("M3_ASCENT pulse_us=");
    Serial.println(pulseUs);
  }

  bool ok = moveMotorTargetsSync(mCmd, pulseUs, progressHook);

  g_busy = false;

  if (!ok) {
    if (!g_hasError) setRobotError("MOVE_SYNC_FAILED");
    Serial.print("MOVE_SYNC_FAILED: ");
    Serial.println(g_lastErrorText);
    return false;
  }

  return true;
}

void initCartesianPoseFromCurrentSteps() {
  MotorMap::MotorTargets mNow;
  mNow.m1 = posSteps_M1;
  mNow.m2 = posSteps_M2;
  mNow.m3 = posSteps_M3;

  Kinematics::Joints qNow;
  if (!MotorMap::motorToJoints(mNow, qNow, g_motorParams)) {
    Serial.println("INIT_CARTESIAN_POSE_FAILED");
    return;
  }

  Kinematics::Pose pNow = Kinematics::robotFK(qNow);
  ScaraPose::init(g_poseState, pNow);
  g_lastControlMs = millis();

  Serial.print("Pose inicial: x=");
  Serial.print(pNow.x, 3);
  Serial.print(" y=");
  Serial.print(pNow.y, 3);
  Serial.print(" z=");
  Serial.println(pNow.z, 3);
}

bool solveIKNearestCurrent(const Kinematics::Pose& poseCmd,
                           Kinematics::Joints& qBest,
                           MotorMap::MotorTargets& mBest) {
  Kinematics::Joints qUp, qDown;
  MotorMap::MotorTargets mUp, mDown;

  bool okUp   = Kinematics::robotIK(poseCmd, qUp,   true);
  bool okDown = Kinematics::robotIK(poseCmd, qDown, false);

  if (!okUp && !okDown) return false;

  long curM1 = posSteps_M1;
  long curM2 = posSteps_M2;
  long curM3 = posSteps_M3;

  auto cost = [&](const MotorMap::MotorTargets& m) -> long {
    return labs(m.m1 - curM1) + labs(m.m2 - curM2) + labs(m.m3 - curM3);
  };

  bool validUp = false;
  bool validDown = false;

  if (okUp) {
    if (MotorMap::jointsToMotor(qUp, mUp, g_motorParams) &&
        MotorMap::withinMotorLimits(mUp, g_motorLimits)) {
      validUp = true;
    }
  }

  if (okDown) {
    if (MotorMap::jointsToMotor(qDown, mDown, g_motorParams) &&
        MotorMap::withinMotorLimits(mDown, g_motorLimits)) {
      validDown = true;
    }
  }

  if (!validUp && !validDown) return false;

  if (validUp && !validDown) {
    qBest = qUp;
    mBest = mUp;
    return true;
  }

  if (!validUp && validDown) {
    qBest = qDown;
    mBest = mDown;
    return true;
  }

  long costUp   = cost(mUp);
  long costDown = cost(mDown);

  if (costUp <= costDown) {
    qBest = qUp;
    mBest = mUp;
  } else {
    qBest = qDown;
    mBest = mDown;
  }

  return true;
}

void processCartesianJoystickControl(int lx, int ly, int ry) {
  if (!g_calibrationDone) return;
  if (!ScaraPose::isInitialized(g_poseState)) return;

  unsigned long now = millis();
  float dt = (g_lastControlMs == 0) ? 0.02f : (now - g_lastControlMs) / 1000.0f;
  g_lastControlMs = now;

  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  ScaraPose::updateFromJoystick(g_poseState, lx, ly, ry, dt);

  Kinematics::Pose poseCmd = ScaraPose::getDesiredPose(g_poseState);

  Safety::SafetyCode poseStatus = Safety::validatePose(poseCmd, g_safetyCfg);
  if (poseStatus != Safety::SafetyCode::OK) {
    Serial.print("Pose invalida: ");
    Serial.println(Safety::toString(poseStatus));
    return;
  }

  Kinematics::Joints qCmd;
  MotorMap::MotorTargets mCmd;

  if (!solveIKNearestCurrent(poseCmd, qCmd, mCmd)) {
    Serial.println("IK_FAILED");
    return;
  }

  Safety::SafetyCode cmdStatus = Safety::validateCommand(
    poseCmd, qCmd, mCmd,
    g_calibrationDone,
    g_motorLimits,
    g_safetyCfg
  );

  if (cmdStatus != Safety::SafetyCode::OK) {
    Serial.print("Movimiento bloqueado: ");
    Serial.println(Safety::toString(cmdStatus));
    return;
  }

  moveCartesianTargets(mCmd);
}

long axisToJogDelta(int8_t v, long maxDelta) {
  int a = abs((int)v);
  if (a < 15) return 0;
  if (maxDelta < 1) maxDelta = 1;

  long d = 1 + ((long)(a - 15) * (maxDelta - 1)) / (127 - 15);
  return (v >= 0) ? d : -d;
}

bool moveToPoseXYZ(float x, float y, float z, MotionProgressHook progressHook) {
  Kinematics::Pose poseCmd;
  poseCmd.x = x;
  poseCmd.y = y;
  poseCmd.z = z;

  Kinematics::Joints qCmd;
  MotorMap::MotorTargets mCmd;

  Safety::SafetyCode poseStatus = Safety::validatePose(poseCmd, g_safetyCfg);
  if (poseStatus != Safety::SafetyCode::OK) {
    setRobotError(Safety::toString(poseStatus));
    Serial.print("GOTO bloqueado: ");
    Serial.println(Safety::toString(poseStatus));
    return false;
  }

  if (!solveIKNearestCurrent(poseCmd, qCmd, mCmd)) {
    setRobotError("IK_FAILED");
    Serial.println("GOTO IK_FAILED");
    return false;
  }

  Safety::SafetyCode cmdStatus = Safety::validateCommand(
    poseCmd, qCmd, mCmd,
    g_calibrationDone,
    g_motorLimits,
    g_safetyCfg
  );

  if (cmdStatus != Safety::SafetyCode::OK) {
    setRobotError(Safety::toString(cmdStatus));
    Serial.print("GOTO invalidado: ");
    Serial.println(Safety::toString(cmdStatus));
    return false;
  }

  if (!moveCartesianTargets(mCmd, progressHook)) {
    return false;
  }

  clearRobotError();
  ScaraPose::setDesiredPose(g_poseState, poseCmd);
  return true;
}

bool moveToJoints(float q1, float q2, float q3) {
  Kinematics::Joints qCmd;
  qCmd.q1 = q1;
  qCmd.q2 = q2;
  qCmd.q3 = q3;

  MotorMap::MotorTargets mCmd;
  if (!MotorMap::jointsToMotor(qCmd, mCmd, g_motorParams)) {
    setRobotError("MOVEJ_INVALID_JOINTS");
    Serial.println("MOVEJ_INVALID_JOINTS");
    return false;
  }

  if (!MotorMap::withinMotorLimits(mCmd, g_motorLimits)) {
    setRobotError("MOTOR_LIMIT_EXCEEDED");
    Serial.println("MOVEJ_LIMIT_EXCEEDED");
    return false;
  }

  if (!moveCartesianTargets(mCmd)) {
    return false;
  }

  clearRobotError();
  initCartesianPoseFromCurrentSteps();
  return true;
}

static long clampLongManual(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void handleManualJoint(const PadState& st) {
  g_stop = false;
  g_cartesianControlEnabled = false;
  g_guiActive = false;
  clearRobotError();
  markControlSource(CONTROL_SOURCE_ESP);

  long dM1 = axisToJogDelta(st.lx, 12);
  long dM2 = axisToJogDelta(st.ly, 12);
  long dM3 = axisToJogDelta(st.ry, MANUAL_M3_MAX_DELTA_STEPS);

  if (dM1 == 0 && dM2 == 0 && dM3 == 0) {
    return;
  }

  MotorMap::MotorTargets target;

  target.m1 = clampLongManual(posSteps_M1 + dM1, M1_minSteps, M1_maxSteps);
  target.m2 = clampLongManual(posSteps_M2 + dM2, M2_minSteps, M2_maxSteps);
  target.m3 = clampLongManual(posSteps_M3 + dM3, M3_MIN_STEPS_PHYS, M3_MAX_STEPS_PHYS);

  uint16_t pulseUs =
      (dM1 == 0 && dM2 == 0 && dM3 != 0)
      ? MANUAL_M3_PULSE_US
      : g_PULSE12_US;

  moveMotorTargetsSync(target, pulseUs);
}

void handleManualCartesian(const PadState& st) {
  g_stop = false;
  g_cartesianControlEnabled = true;
  g_guiActive = false;
  clearRobotError();
  markControlSource(CONTROL_SOURCE_ESP);
  processCartesianJoystickControl(st.lx, st.ly, st.ry);
}

static bool endstopWouldBlock(uint8_t motor, bool positive) {
  if (motor == 1) {
    if (positive && DI_TOTAL > 2 && endstopActiveConfirmed(DI_PINS[2])) {
      Serial.print("ENDSTOP_CONFIRMED motor=M1 dir=POS di=");
      Serial.println(DI_PINS[2]);
      return true;
    }
    if (!positive && DI_TOTAL > 3 && endstopActiveConfirmed(DI_PINS[3])) {
      Serial.print("ENDSTOP_CONFIRMED motor=M1 dir=NEG di=");
      Serial.println(DI_PINS[3]);
      return true;
    }
  }

  if (motor == 2) {
    if (positive && DI_TOTAL > 0 && endstopActiveConfirmed(DI_PINS[0])) {
      Serial.print("ENDSTOP_CONFIRMED motor=M2 dir=POS di=");
      Serial.println(DI_PINS[0]);
      return true;
    }
    if (!positive && DI_TOTAL > 1 && endstopActiveConfirmed(DI_PINS[1])) {
      Serial.print("ENDSTOP_CONFIRMED motor=M2 dir=NEG di=");
      Serial.println(DI_PINS[1]);
      return true;
    }
  }

  if (motor == 3) {
    if (!positive && DI_TOTAL > 6 && endstopActiveConfirmed(DI_PINS[6])) {
      Serial.print("ENDSTOP_CONFIRMED motor=M3 dir=NEG di=");
      Serial.println(DI_PINS[6]);
      return true;
    }
  }

  return false;
}

bool moveMotorTargetsSync(const MotorMap::MotorTargets& target, uint16_t pulse_us,
                         MotionProgressHook progressHook) {
  MotorMap::MotorTargets t;

  t.m1 = clampLongManual(target.m1, M1_minSteps, M1_maxSteps);
  t.m2 = clampLongManual(target.m2, M2_minSteps, M2_maxSteps);
  t.m3 = clampLongManual(target.m3, M3_MIN_STEPS_PHYS, M3_MAX_STEPS_PHYS);

  long d1 = t.m1 - posSteps_M1;
  long d2 = t.m2 - posSteps_M2;
  long d3 = t.m3 - posSteps_M3;

  long a1 = labs(d1);
  long a2 = labs(d2);
  long a3 = labs(d3);

  long ticks = max(a1, max(a2, a3));
  if (ticks <= 0) return true;

  // Toda salida fallida de un movimiento con XY invalida su referencia.
  // Una cancelacion solo Z no invalida M1/M2; un ESTOP lo hace por separado.
  struct ArmReferenceGuard {
    bool movesXY;
    bool completed;
    ~ArmReferenceGuard() {
      if (movesXY && !completed) {
        g_armPositionTrusted = false;
        g_positionTrusted = false;
        g_calibrationDone = false;
        g_calDone = false;
        g_stop = true;
      }
    }
  } armReferenceGuard = {a1 > 0 || a2 > 0, false};

  const uint32_t moveStartMs = millis();
  Serial.print("MOVE_SYNC_PLAN d1="); Serial.print(d1);
  Serial.print(" d2="); Serial.print(d2);
  Serial.print(" d3="); Serial.print(d3);
  Serial.print(" ticks="); Serial.print(ticks);
  Serial.print(" pulse_us="); Serial.println(pulse_us);

  bool p1 = d1 > 0;
  bool p2 = d2 > 0;
  bool p3 = d3 > 0;

  PinStatus dir1 = p1 ? dirPosFromNeg(DIR1_LEVEL_NEG_CFG) : DIR1_LEVEL_NEG_CFG;
  PinStatus dir2 = p2 ? dirPosFromNeg(DIR2_LEVEL_NEG_CFG) : DIR2_LEVEL_NEG_CFG;
  PinStatus dir3 = p3 ? dirPosFromNeg(DIR3_LEVEL_NEG_CFG) : DIR3_LEVEL_NEG_CFG;

  MachineControl_DigitalOutputs.write(STEP1_CH_CFG, LOW);
  MachineControl_DigitalOutputs.write(STEP2_CH_CFG, LOW);
  MachineControl_DigitalOutputs.write(STEP3_CH_CFG, LOW);

  delayMicroseconds(5);

  MachineControl_DigitalOutputs.write(DIR1_CH_CFG, dir1);
  MachineControl_DigitalOutputs.write(DIR2_CH_CFG, dir2);
  MachineControl_DigitalOutputs.write(DIR3_CH_CFG, dir3);

  delayMicroseconds(DIR_SETUP_US_SYNC);

  long acc1 = 0;
  long acc2 = 0;
  long acc3 = 0;
  bool m3ZeroRebased = false;

  uint32_t lastProgressMs = millis();
  if (progressHook && !progressHook()) return false;

  for (long i = 0; i < ticks; i++) {
    if (g_stop || g_remoteEstop || pollRemoteEstopDuringBlockingMotion()) return false;
    // STEP esta en LOW aqui. Atender el encoder/cierre sin alargar un pulso.
    if (progressHook && millis() - lastProgressMs >= 20) {
      lastProgressMs = millis();
      if (!progressHook()) return false;
    }

    bool s1 = false;
    bool s2 = false;
    bool s3 = false;

    if (a1 > 0) {
      acc1 += a1;
      if (acc1 >= ticks) {
        acc1 -= ticks;
        s1 = true;
      }
    }

    if (a2 > 0) {
      acc2 += a2;
      if (acc2 >= ticks) {
        acc2 -= ticks;
        s2 = true;
      }
    }

    if (a3 > 0) {
      acc3 += a3;
      if (acc3 >= ticks) {
        acc3 -= ticks;
        s3 = true;
      }
    }

    if (s1 && endstopWouldBlock(1, p1)) {
      setRobotError("M1_ENDSTOP");
      return false;
    }

    if (s2 && endstopWouldBlock(2, p2)) {
      setRobotError("M2_ENDSTOP");
      return false;
    }

    // Leer DI06 en cada paso agregaba una sobrecarga considerable al descenso.
    // Cada 16 pasos limita el sobrepaso a unas decimas de milimetro y se
    // conserva una lectura al inicio y en el ultimo tick del movimiento.
    const bool checkM3MinEndstop =
        s3 && !p3 && (((i & 0x0FL) == 0) || (i + 1 == ticks));

    if (checkM3MinEndstop && endstopWouldBlock(3, p3)) {
      // El switch confirmado define el mismo cero fisico; no desplazar el
      // limite maximo ni los pasos nominales de HOME al corregir el contador.
      const bool globalRezero = M3_REZERO_ON_ENDSTOP && !g_stop &&
          !g_estopActive && !g_remoteEstop;
      if (!p3 && (t.m3 == M3_MIN_STEPS_PHYS || globalRezero)) {
        // DI06 define fisicamente la posicion de pick (M3=0). Si aparece
        // durante un movimiento cuyo objetivo es ese mismo limite, se toma
        // como llegada valida y M1/M2 pueden terminar su alineacion.
        rebaseM3ZeroFromSwitch();
        m3ZeroRebased = true;
        // El jog articular expresa un incremento, no una altura absoluta:
        // al llegar al switch quedarse en cero, sin invertir el joystick.
        const bool jointJog = g_controlSource == CONTROL_SOURCE_ESP &&
            g_lastManualMode == MANUAL_MODE_JOINT && !g_homeTransitionActive;
        if (t.m3 > M3_MIN_STEPS_PHYS && !jointJog) {
          // El objetivo era hover/place, no Z=0. Detener esta trayectoria:
          // primero recuperar la altura con XY quieto, luego terminar XY.
          // Nunca devolver exito dejando el robot abajo y la pose en hover.
          MotorMap::MotorTargets restoreHeight = t;
          restoreHeight.m1 = posSteps_M1;
          restoreHeight.m2 = posSteps_M2;
          Serial.println("M3_ZERO_RESTORE_HEIGHT");
          const uint16_t restorePulseUs = selectAutoCoordinatedPulseUs(
              restoreHeight, g_calibrating ? g_PULSE3_US : M3_ASCENT_SYNC_PULSE_US);
          if (!moveMotorTargetsSync(restoreHeight, restorePulseUs)) {
            return false;
          }
          if (g_stop || g_estopActive || g_remoteEstop ||
              pollRemoteEstopDuringBlockingMotion()) return false;
          if (endstopActiveConfirmed(DI_PINS[6])) {
            setRobotError("M3_ZERO_SWITCH_STUCK");
            // No aceptar una pose Z positiva si el switch sigue indicando cero.
            // M1/M2 no se movieron en la recuperacion solo Z y mantienen su
            // referencia, salvo que el guard del movimiento XY invalide esta.
            g_positionTrusted = false;
            g_calibrationDone = false;
            g_calDone = false;
            g_stop = true;
            return false;
          }
          // El movimiento restante es XY; conservar el ritmo especial HOME.
          const uint16_t remainingPulseUs = g_homeTransitionActive
              ? g_PULSE12_US : CARTESIAN_SYNC_PULSE_US;
          if (!moveMotorTargetsSync(t, remainingPulseUs, progressHook)) {
            return false;
          }
          armReferenceGuard.completed = true;
          Serial.println("M3_ZERO_RESTORE_DONE");
          Serial.print("MOVE_SYNC_DONE elapsed_ms=");
          Serial.println(millis() - moveStartMs);
          return true;
        }
        a3 = 0;
        acc3 = 0;
        s3 = false;
        Serial.println("M3 PICK ENDSTOP OK. posSteps_M3=0");
        // En un descenso solo M3 no queda ningun eje pendiente: no consumir
        // ticks vacios con la garra ya abajo. Si hay XY, permitir que terminen.
        if (a1 == 0 && a2 == 0) break;
      } else {
        setRobotError("M3_ENDSTOP");
        return false;
      }
    }

    if (s1) MachineControl_DigitalOutputs.write(STEP1_CH_CFG, HIGH);
    if (s2) MachineControl_DigitalOutputs.write(STEP2_CH_CFG, HIGH);
    if (s3) MachineControl_DigitalOutputs.write(STEP3_CH_CFG, HIGH);

    delayMicroseconds(pulse_us);

    if (s1) {
      MachineControl_DigitalOutputs.write(STEP1_CH_CFG, LOW);
      posSteps_M1 += p1 ? 1 : -1;
    }

    if (s2) {
      MachineControl_DigitalOutputs.write(STEP2_CH_CFG, LOW);
      posSteps_M2 += p2 ? 1 : -1;
    }

    if (s3) {
      MachineControl_DigitalOutputs.write(STEP3_CH_CFG, LOW);
      posSteps_M3 += p3 ? 1 : -1;
    }

    delayMicroseconds(pulse_us);
  }

  if (progressHook && !progressHook()) return false;
  if (m3ZeroRebased) syncPoseAfterM3Zero();
  Serial.print("MOVE_SYNC_DONE elapsed_ms=");
  Serial.println(millis() - moveStartMs);
  armReferenceGuard.completed = true;
  return true;
}
