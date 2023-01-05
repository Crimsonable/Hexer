#pragma once
#define HEXER_LOG
#define HEXER_INLINE __forceinline

namespace Hexer {
using uint = unsigned int;
using ID = size_t;

enum class Device { CPU, GPU };
enum class MeshType { Triangle = 3, Quads = 4 };

template <Device device, typename T> struct MemoryManager;

template <Device device,
          template <Device device, typename ParamListTp> class Derived,
          typename ParamTuple = std::tuple<>>
class CrtpExprBase;
} // namespace Hexer