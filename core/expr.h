#pragma once
#include "base.h"
#include "traits.h"

#include <string_view>
#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

template <Meta::ConceptNonExpr T, typename... Args>
HEXER_INLINE auto Eval(T &&val, Args &&...args) {
  return Meta::ref_helper(std::forward_as_tuple(val));
}

template <Meta::ConceptExpr T, typename... Args>
HEXER_INLINE auto Eval(T &&expr, Args &&...args) {
  if constexpr (Meta::ConceptTuple<decltype(expr.execute(
                    std::forward<Args>(args)...))>)
    return Meta::ref_helper(expr.execute(std::forward<Args>(args)...));
  else
    return Meta::ref_helper(
        std::forward_as_tuple(expr.execute(std::forward<Args>(args)...)));
}

template <typename ParamTuple, size_t... I, typename... Args>
HEXER_INLINE auto tuple_eval_impl(ParamTuple &&tp,
                                  const std::index_sequence<I...> &,
                                  Args &&...args) {
  return std::tuple_cat(Eval(
      std::forward<std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>(
          std::get<I>(tp)),
      std::forward<Args>(args)...)...);
}

template <typename ParamTuple, typename... Args>
HEXER_INLINE auto tuple_eval(ParamTuple &&tp, Args &&...args) {
  return tuple_eval_impl(
      tp,
      std::make_index_sequence<
          std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{},
      std::forward<Args>(args)...);
}

template <Device device,
          template <Device device, typename ParamListTp> class Derived,
          typename ParamTuple>
class CrtpExprBase {

  using _Derived = Derived<device, ParamTuple>;

public:
  std::remove_reference_t<ParamTuple> _args;

public:
  template <typename ArgsTuple, size_t... I>
  HEXER_INLINE auto execute_helper(const std::index_sequence<I...> &,
                                   ArgsTuple &&args_tuple) {
    // return Derived::eval(
    //     std::forward<std::tuple_element_t<I,
    //     std::remove_cvref_t<ArgsTuple>>>(
    //         std::get<I>(args_tuple))...);
    // return Derived::eval(std::get<I>(args_tuple)...);
    return derived()->eval(
        std::forward<std::tuple_element_t<I, std::remove_cvref_t<ArgsTuple>>>(
            std::get<I>(args_tuple))...);
  }

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(ParamTuple &params) : _args(std::move(params)) {}

  static void ExprFeature() {}

  constexpr bool check() { return Meta::CheckExprInTuple(_args); }

  HEXER_INLINE
  Derived<device, ParamTuple> *derived() {
    return static_cast<Derived<device, ParamTuple> *>(this);
  }

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
    return Derived<device, decltype(new_args)>(new_args);
  }

  template <Meta::ConceptExpr Expr> auto operator|(Expr &&exp) {
    return exp(*this);
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class OpBase : public CrtpExprBase<device, OpBase, ParamTuple> {
public:
};
} // namespace Hexer