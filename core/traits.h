#pragma once
#include "base.h"

#include <tuple>

namespace Hexer {
namespace Meta {

template <typename T>
concept ConceptExpr = requires { std::remove_cvref_t<T>::ExprFeature; };

template <typename T>
concept ConceptNonExpr = !
ConceptExpr<T>;

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

template <typename Ret, typename... Args> struct traits<Ret (*)(Args...)> {
  static constexpr int param_size = sizeof...(Args);
  using ArgsTypes = std::tuple<Args...>;
};

template <typename Func, typename Class, size_t... I, typename ParamTuple>
inline auto tuple_apply_impl(Func &&f, Class *self,
                             const std::index_sequence<I...> &,
                             ParamTuple &&tp) {
  return (self->*f)(
      std::forward<std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>(
          std::get<I>(tp))...);
}

template <typename Func, typename Class, typename ParamTuple>
inline auto tuple_apply(Func &&f, Class *self, ParamTuple &&tp) {
  return tuple_apply_impl(
      std::forward<Func>(f), self,
      std::make_index_sequence<
          std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{},
      std::forward<decltype(tp)>(tp));
}

template <typename Func, size_t... I, typename ParamTuple>
inline auto tuple_apply_impl(Func &&f, const std::index_sequence<I...> &,
                             ParamTuple &&tp) {
  return f(
      std::forward<std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>(
          std::get<I>(tp))...);
}

template <typename Func, typename ParamTuple>
inline auto tuple_apply(Func &&f, ParamTuple &&tp) {
  return tuple_apply_impl(
      std::forward<Func>(f),
      std::make_index_sequence<
          std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{},
      std::forward<decltype(tp)>(tp));
}

template <typename ParamTuple, size_t... I>
constexpr bool CheckExprInTuple_Impl(const std::index_sequence<I...> &) {
  bool flag = false;
  flag =
      ((flag = flag ||
               (ConceptExpr<
                   std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>)),
       ...);
  return flag;
}

template <typename ParamTuple> constexpr bool CheckExprInTuple() {
  if constexpr (std::is_same_v<std::remove_cvref_t<ParamTuple>, std::tuple<>>)
    return true;
  else
    return CheckExprInTuple_Impl<ParamTuple>(
        std::make_index_sequence<
            std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{});
}

template <typename T> auto ref_helper(T &&t) {
  if constexpr (std::is_lvalue_reference_v<T>)
    return std::ref(std::forward<T>(t));
  else
    return std::forward<T>(t);
}
} // namespace Meta
} // namespace Hexer