#pragma once
#define HEXER_LOG
#define HEXER_INLINE __forceinline

#ifdef __DEBUG
    #include <chrono>
    #define HEXER_TIMER(func) \
    auto t1=std::chrono::steady_clock::now();
    func;
    auto t2=std::chrono::steady_clock::now();
    

#endif

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