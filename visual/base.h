#pragma once

#include <Expblas/ftensor.h>
#include <math.h>

namespace Visual {
using vec3f = Expblas::FTensor<float, 3>;
using vec2f = Expblas::FTensor<float, 2>;
using vec4f = Expblas::FTensor<float, 4>;
using mat2f = Expblas::FTensor<float, 2, 2>;
using mat3f = Expblas::FTensor<float, 3, 3>;
using mat4f = Expblas::FTensor<float, 4, 4>;

template <typename T> inline T radians(const T &angle) {
  return angle / 180.0 * M_PI;
}
} // namespace Visual