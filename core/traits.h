#pragma once
#include "base.h"

namespace Hexer {
namespace Meta {
template <typename Type> struct traits;

template <Device device, typename Alloctor, typename EleDataType>
struct traits<OpBase<device, Alloctor, EleDataType>> {
  using _Alloctor = Alloctor;
  using _EleDataType = EleDataType;
};

template <typename Ret, typename Class, typename... Args>
struct traits<Ret (Class::*)(Args...)> {
  static constexpr int param_size = sizeof...(Args);
};

template <typename Func, size_t... I, typename... Args>
inline auto tuple_apply_impl(Func &&f, const std::index_sequence<I...> &,
                             const std::tuple<Args...> &tp) {
  return f(std::get<I>(tp)...);
}

template <typename Func, typename... Args>
inline auto tuple_apply(Func &&f, const std::tuple<Args...> &tp) {
  return tuple_apply_impl(std::forward<Func>(f),
                          std::make_index_sequence<sizeof...(Args)>{}, tp);
}
} // namespace Meta
} // namespace Hexer