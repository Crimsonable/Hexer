#pragma once
#define HEXER_LOG
#define HEXER_INLINE __forceinline

#include <chrono>
#include <spdlog/spdlog.h>

template <typename Functor> void hexer_timer(Functor &&f, const char *desc) {
  auto t1 = std::chrono::steady_clock::now();
  f();
  auto t2 = std::chrono::steady_clock::now();
  spdlog::info(
      "Total time cost: {:03.5f}ms",
      double(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()));
}

#include <omp.h>

namespace Hexer {
using uint = unsigned int;
using ID = size_t;

enum class Device { CPU = 1, GPU = 3 };
enum class MeshType { Triangle = 3, Quads = 4 };

template <Device device, typename T> struct MemoryManager;

template <Device device,
          template <Device device, typename ParamListTp> class Derived,
          typename ParamTuple = std::tuple<>>
class CrtpExprBase;
} // namespace Hexer