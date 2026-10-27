#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

namespace Kinematics {

// =========================
// Estructuras de datos
// =========================
struct Pose {
  float x;    // mm
  float y;    // mm
  float z;    // mm
};

struct Joints {
  float q1;   // rad
  float q2;   // rad
  float q3;   // mm
};

struct RobotParams {
  float h_base;   // mm
  float d1;       // mm
  float L1;       // mm
  float L2;       // mm
  float dtool;    // mm

  float q3_min;   // mm
  float q3_max;   // mm
};

// =========================
// Parámetros por defecto
// =========================
RobotParams defaultParams();

// =========================
// Utilidades
// =========================
float wrapAngle(float a);
bool isFinitePose(const Pose& p);
bool isFiniteJoints(const Joints& q);

// =========================
// Cinemática directa
// =========================
Pose robotFK(const Joints& q, const RobotParams& p = defaultParams());

// =========================
// Cinemática inversa
// elbowUp = true  -> solución codo arriba
// elbowUp = false -> solución codo abajo
// =========================
bool robotIK(const Pose& pose, Joints& qOut, bool elbowUp = true,
             const RobotParams& p = defaultParams());

// =========================
// Validaciones
// =========================
bool withinQ3Limits(float q3, const RobotParams& p = defaultParams());
bool withinWorkspaceXY(float x, float y, const RobotParams& p = defaultParams());

}  // namespace Kinematics

#endif