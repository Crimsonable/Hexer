#pragma once
#include <math.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace Visual {
template <typename T> inline T radians(const T &angle) {
  return angle / 180.0 * M_PI;
}
} // namespace Visual