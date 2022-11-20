#pragma once
#include "base.h"

#include <tuple>

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
  using ArgsTypes = std::tuple<Args...>;
};

template <typename Func, typename Class, size_t... I, typename... Args>
inline auto tuple_apply_impl(Func &&f, Class *self,
                             const std::index_sequence<I...> &,
                             std::tuple<Args...> &tp) {
  return (self->*f)(std::forward<std::tuple_element_t<I, std::tuple<Args...>>>(
      std::get<I>(tp))...);
}

template <typename Func, typename Class, typename... Args>
inline auto tuple_apply(Func &&f, Class *self, std::tuple<Args...> &tp) {
  return tuple_apply_impl(std::forward<Func>(f), self,
                          std::make_index_sequence<sizeof...(Args)>{}, tp);
}

template <typename T> auto ref_helper(T &&t) {
  if constexpr (std::is_lvalue_reference_v<T>)
    return std::ref(std::forward<T>(t));
  else
    return std::forward<T>(t);
}
} // namespace Meta
} // namespace Hexer