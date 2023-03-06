#pragma once
#include "base.h"
#include "traits.h"

#include <string_view>
#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

template <Meta::ConceptNonExpr T> HEXER_INLINE auto Eval(T &&val) {
  return Meta::ref_helper(std::forward_as_tuple(val));
}

template <Meta::ConceptExpr T> HEXER_INLINE auto Eval(T &&expr) {
  // if constexpr (Meta::ConceptTuple<decltype(expr.execute())>)
  //   return expr.execute();
  // else
  //   return std::move(std::forward_as_tuple(std::move(expr.execute())));
  return std::make_tuple(std::move(expr.execute()));
}

template <typename ParamTuple, size_t... I>
HEXER_INLINE auto tuple_eval_impl(ParamTuple &&tp,
                                  const std::index_sequence<I...> &) {
  return std::tuple_cat(Eval(
      std::forward<std::tuple_element_t<I, std::remove_cvref_t<ParamTuple>>>(
          std::get<I>(tp)))...);
}

template <typename ParamTuple> HEXER_INLINE auto tuple_eval(ParamTuple &&tp) {
  return tuple_eval_impl(
      tp, std::make_index_sequence<
              std::tuple_size_v<std::remove_cvref_t<ParamTuple>>>{});
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
    return derived()->eval(
        std::forward<std::tuple_element_t<I, std::remove_cvref_t<ArgsTuple>>>(
            std::get<I>(args_tuple))...);
  }

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(ParamTuple &params) : _args(std::move(params)) {}

  static void ExprFeature() {}

  constexpr bool check() { return Meta::CheckExprInTuple(_args); }

  template <int N, typename... Args> auto feed(Args &&...args) {}

  HEXER_INLINE
  Derived<device, ParamTuple> *derived() {
    return static_cast<Derived<device, ParamTuple> *>(this);
  }

  template <typename... Args> auto execute(Args &&...args) {
    if constexpr (std::tuple_size<ParamTuple>::value &&
                  Meta::CheckExprInTuple<ParamTuple>()) {
      return execute_helper(
          std::make_index_sequence<
              std::tuple_size_v<decltype(tuple_eval(_args))>>{},
          tuple_eval(_args));
    } else if constexpr (sizeof...(Args)) {
      auto tp = std::tuple_cat(_args, std::make_tuple(Meta::ref_helper(
                                          std::forward<Args>(args))...));
      return execute_helper(
          std::make_index_sequence<
              std::tuple_size_v<std::remove_cvref_t<decltype(tp)>>>{},
          tp);
    } else {
      return execute_helper(
          std::make_index_sequence<std::tuple_size_v<ParamTuple>>{}, _args);
    }
  }

  template <typename... Args> auto operator()(Args &&...args) {
    auto new_args = std::tuple_cat(
        std::make_tuple(Meta::ref_helper(std::forward<Args>(args))...), _args);
    return Derived<device, decltype(new_args)>(new_args);
  }

  // template <Meta::ConceptExpr Expr> auto operator|(Expr &&exp) {
  //   return exp(*this);
  // }
};

template <typename Expr1, typename Expr2>
  requires(Meta::ConceptExpr<Expr1> && Meta::ConceptExpr<Expr2>)
auto operator|(Expr1 &&exp1, Expr2 &&exp2) {
  return exp2(exp1);
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class OpBase : public CrtpExprBase<device, OpBase, ParamTuple> {
public:
};
} // namespace Hexer