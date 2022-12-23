#pragma once
#include "base.h"
#include "traits.h"

#include <string_view>
#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

template <Meta::ConceptNonExpr T, typename... Args>
inline auto Eval(T &&val, Args &&...args) {
  return Meta::ref_helper(std::forward<T>(val));
}

template <Meta::ConceptExpr T, typename... Args>
inline auto Eval(T &&expr, Args &&...args) {
  return Meta::ref_helper(expr.execute(std::forward<Args>(args)...));
}

template <typename ParamTuple, size_t... I, typename... Args>
inline auto tuple_eval_impl(ParamTuple &&tp, const std::index_sequence<I...> &,
                            Args &&...args) {
  return std::make_tuple(Eval(
      std::forward<std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>(
          std::get<I>(tp)),
      std::forward<Args>(args)...)...);
}

template <typename ParamTuple, typename... Args>
inline auto tuple_eval(ParamTuple &&tp, Args &&...args) {
  return tuple_eval_impl(
      tp,
      std::make_index_sequence<
          std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{},
      std::forward<Args>(args)...);
}

template <Device device, typename Derived, typename ParamTuple>
class CrtpExprBase {
private:
  ParamTuple _args;

private:
  template <typename ArgsTuple, size_t... I>
  auto execute_helper(const std::index_sequence<I...> &,
                      ArgsTuple &&args_tuple) {
    return Derived::eval(
        std::forward<std::tuple_element_t<I, std::remove_cvref_t<ArgsTuple>>>(
            std::get<I>(args_tuple))...);
  }

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(ParamTuple &params) : _args(params) {}

  Derived *cast_to() { return static_cast<Derived *>(this); }

  static void ExprFeature() {}

  constexpr bool check() { return Meta::CheckExprInTuple(_args); }

  template <typename... Args> auto execute(Args &&...args) {
    if constexpr (std::tuple_size<ParamTuple>::value &&
                  Meta::CheckExprInTuple<ParamTuple>()) {
      // return Meta::tuple_apply(&Derived::eval,
      //                          tuple_eval(_args,
      //                          std::forward<Args>(args)...));
      return execute_helper(
          std::make_index_sequence<std::tuple_size_v<decltype(tuple_eval(
              _args, std::forward<Args>(args)...))>>{},
          tuple_eval(_args, std::forward<Args>(args)...));
    } else if constexpr (sizeof...(Args)) {
      // return Meta::tuple_apply(
      //     &Derived::eval,
      //     std::tuple_cat(_args, std::make_tuple(Meta::ref_helper(
      //                               std::forward<Args>(args))...)));
      auto tp = std::tuple_cat(_args, std::make_tuple(Meta::ref_helper(
                                          std::forward<Args>(args))...));
      return execute_helper(
          std::make_index_sequence<
              std::tuple_size_v<std::remove_cvref_t<decltype(tp)>>>{},
          tp);
    } else {
      // return Meta::tuple_apply(&Derived::eval, _args);
      return execute_helper(
          std::make_index_sequence<std::tuple_size_v<ParamTuple>>{}, _args);
    }
  }

  template <typename... Args> auto operator()(Args &&...args) {
    auto new_args = std::tuple_cat(
        _args, std::make_tuple(Meta::ref_helper(std::forward<Args>(args))...));
    // static_assert(
    //     std::tuple_size_v<decltype(new_args)> <=
    //         Meta::traits<decltype(&Derived::template eval)>::param_size,
    //     "parameters oversize.");
    return CrtpExprBase<device, Derived, decltype(new_args)>(new_args);
  }

  template <Meta::ConceptExpr Expr> auto operator|(Expr &&exp) {
    return exp(*this);
  }
};

template <Device device, typename Alloctor, typename EleDataType>
class OpBase
    : public CrtpExprBase<device, OpBase<device, Alloctor, EleDataType>> {
public:
};

template <Device device, typename ContainerT>
class Stream : public CrtpExprBase<device, Stream<device, ContainerT>,
                                   std::tuple<ContainerT>> {
public:
  Stream(ContainerT &&data)
      : CrtpExprBase<device, Stream<device, ContainerT>,
                     std::tuple<ContainerT>>(std::tuple<ContainerT>{data}) {}
};

} // namespace Hexer