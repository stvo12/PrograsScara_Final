#pragma once

namespace PickReference {
// Proyeccion del desplazamiento XY sobre el sentido de avance de la banda.
constexpr float longitudinal(float nominal, float offsetX, float offsetY,
                             float beltCos, float beltSin) {
  return nominal + offsetX * beltCos + offsetY * beltSin;
}
}
