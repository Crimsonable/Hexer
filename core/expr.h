#include "base.h"
#include "hex_memory.h"
#include "traits.h"

#include <string_view>
#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

// template <typename T> inline auto Eval(T &&val) {
//   return Meta::ref_helper(std::forward<T>(val));
// }

// template <Device device, typename Derived, typename ParamTuple>
// inline auto Eval(CrtpExprBase<device, Derived, ParamTuple> &expr) {
//   return Meta::ref_helper(expr.execute());
// }

// template <size_t... I, typename... Args>
// inline auto tuple_eval_impl(const std::index_sequence<I...> &,
//                             std::tuple<Args...> &tp) {
//   return std::make_tuple(
//       Eval(std::forward<std::tuple_element_t<I, std::tuple<Args...>>>(
//           std::get<I>(tp)))...);
// }

// template <typename... Args> inline auto tuple_eval(std::tuple<Args...> &tp) {
//   return tuple_eval_impl(std::make_index_sequence<sizeof...(Args)>{}, tp);
// }

// template <Device device, typename Derived>
// class CrtpExprBase<device, Derived, void> {
// public:
//   Derived *cast_to() { return static_cast<Derived *>(this); }
// };

template <Device device, typename Derived, typename ParamTuple>
class CrtpExprBase {
private:
  ParamTuple _args;

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(ParamTuple &params) : _args(params) {}

  Derived *cast_to() { return static_cast<Derived *>(this); }

  auto execute() {
    return Meta::tuple_apply(&Derived::eval, cast_to(), tuple_eval(_args));
  }

  template <typename... Args> auto operator()(Args &&...args) {
    auto new_args = std::tuple_cat(
        _args, std::make_tuple(Meta::ref_helper(std::forward<Args>(args))...));
    using p_type = decltype(new_args);
    static_assert(std::tuple_size_v<p_type> <=
                      Meta::traits<decltype(&Derived::eval)>::param_size,
                  "parameters oversize.");
    return CrtpExprBase<device, Derived, p_type>(new_args);
  }
};

template <Device device, typename Alloctor, typename EleDataType>
class OpBase
    : public CrtpExprBase<device, OpBase<device, Alloctor, EleDataType>> {
public:
  int eval(int a) { return a; }
};
} // namespace Hexer