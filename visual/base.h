#pragma once
#include <math.h>

namespace Visual {
template <typename T> inline T radians(const T &angle) {
  return angle / 180.0 * M_PI;
}
} // namespace Visual