#pragma once
#define HEXER_LOG
#define HEXER_INLINE __forceinline

namespace Hexer {
using uint = unsigned int;
using ID = size_t;

enum class Device { CPU, GPU };

template <Device device, typename T> struct MemoryManager;

template <Device device, typename Derived, typename PreExpr = void,
          typename ParamTuple = std::tuple<>>
class CrtpExprBase;
template <Device device, typename Alloctor, typename EleDataType> class OpBase;

} // namespace Hexer