#include "base.h"
#include <memory>

namespace Hexer {
template <typename T> struct MemoryManager<Device::CPU, T> {
  void alloc_normal(void *&target, size_t size) { target = new T[size]; }
};
} // namespace Hexer