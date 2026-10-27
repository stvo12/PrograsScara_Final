#pragma once

#include <math.h>
#include <stdint.h>

// All angles in degrees. zeroXOffsetDeg is gripper X relative to the last
// link at command s0 (0 for X aligned, -90 for Y aligned, modulo 180).
// gain = physical gripper degrees / commanded servo degree, with sign.
inline bool solveCompensatedRotation(float linkDeg, float desiredXDeg,
                                     float s0, float gain, float zeroXOffsetDeg,
                                     uint8_t minAngle,
                                     uint8_t maxAngle, float referenceAngle,
                                     uint8_t& command) {
  if (!isfinite(linkDeg) || !isfinite(desiredXDeg) || !isfinite(s0) ||
      !isfinite(gain) || !isfinite(zeroXOffsetDeg) ||
      !isfinite(referenceAngle) || fabsf(gain) < 0.001f ||
      minAngle > maxAngle || maxAngle > 180) return false;

  const float base = s0 + (desiredXDeg - linkDeg - zeroXOffsetDeg) / gain;
  const float period = fabsf(180.0f / gain);
  if (!isfinite(base) || !isfinite(period)) return false;
  const float first = ceilf((minAngle - base) / period);
  const float last = floorf((maxAngle - base) / period);
  if (first > last) return false;

  // Choose the feasible parallel orientation closest to the last Auto command.
  float n = roundf((referenceAngle - base) / period);
  if (n < first) n = first;
  if (n > last) n = last;
  const float servoDeg = base + n * period;
  const long rounded = lroundf(servoDeg);
  if (rounded < minAngle || rounded > maxAngle) return false;
  command = static_cast<uint8_t>(rounded);
  return true;
}
