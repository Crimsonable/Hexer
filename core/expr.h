#include "base.h"
#include "hex_memory.h"
#include "traits.h"

#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

template <Device device, typename Derived>
class CrtpExprBase<device, Derived, void, void> {
public:
  Derived *cast_to() { return static_cast<Derived *>(this); }

  template <typename... Args> auto eval(Args &&...args) {
    return cast_to()->run(std::forward<Args>(args)...);
  }
};

template <Device device, typename Derived, typename ParamTuple>
class CrtpExprBase<device, Derived, void, ParamTuple> {
private:
  ParamTuple _args;

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(ParamTuple &&params) : _args(params) {}

  Derived *cast_to() { return static_cast<Derived *>(this); }

  template <typename... Args> auto eval(Args &&...args) {
    return cast_to()->run(std::forward<Args>(args)...);
  }

  auto getArgs() { return _args; }

  template <typename... Args> auto operator()(Args &&...args) {
    auto new_args = std::tuple_cat(
        _args, std::make_tuple(Meta::ref_helper(std::forward<Args>(args))...));
    using p_type = decltype(new_args);
    if constexpr (std::tuple_size_v<p_type> <
                  Meta::traits<decltype(&Derived::eval)>::param_size)
      return CrtpExprBase<device, Derived, void, p_type>(new_args);
    else
      return Meta::tuple_apply(&Derived::eval, cast_to(), new_args);
  }

  template <typename Rhs_Derived, typename PTuple>
  auto operator|(CrtpExprBase<device, Rhs_Derived, void, PTuple> &&exp) {
    return CrtpExprBase<device, Rhs_Derived, decltype(*this), PTuple>(
        *this, exp.getArgs())
  }
};

template <Device device, typename Derived, typename PreExpr,
          typename ParamTuple>
class CrtpExprBase {
private:
  ParamTuple _args;
  PreExpr _pre;

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(PreExpr &&pre_expr, ParamTuple &&params)
      : _pre(std::forward<PreExpr>(pre_expr)), _args(params) {}

  Derived *cast_to() { return static_cast<Derived *>(this); }

  template <typename... Args> auto eval(Args &&...args) {
    return cast_to()->run(std::forward<Args>(args)...);
  }

  auto getArgs() { return _args; }

  template <typename... Args> auto operator()(Args &&...args) {
    auto new_args = std::tuple_cat(
        _args, std::make_tuple(Meta::ref_helper(std::forward<Args>(args))...));
    using p_type = decltype(new_args);
    if constexpr (std::tuple_size_v<p_type> <
                  Meta::traits<decltype(&Derived::eval)>::param_size)
      return CrtpExprBase<device, Derived, PreExpr, p_type>(_pre, new_args);
    else
      return Meta::tuple_apply(&Derived::eval, cast_to(), new_args);
  }

  template <typename Rhs_Derived, typename PTuple>
  auto operator|(CrtpExprBase<device, Rhs_Derived, void, PTuple> &&exp) {
    return CrtpExprBase<device, Rhs_Derived, decltype(*this), PTuple>(
        *this, exp.getArgs())
  }
};

template <Device device, typename Alloctor, typename EleDataType>
class OpBase
    : public CrtpExprBase<device, OpBase<device, Alloctor, EleDataType>> {
public:
  int eval(int a) { return a; }
};
} // namespace Hexer