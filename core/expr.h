#include "base.h"
#include "hex_memory.h"
#include "traits.h"

#include <tuple>

namespace Hexer {

enum class MemPolicy { INPLACE, OUTPLACE };

template <Device device, typename Derived, typename ParamTuple = std::tuple<>>
class CrtpExprBase {
private:
  ParamTuple _args;

public:
  CrtpExprBase() : _args(std::tuple<>()) {}

  CrtpExprBase(const ParamTuple &params) : _args(params) {}

  Derived *cast_to() { return static_cast<Derived *>(this); }

  template <typename... Args> auto eval(Args &&...args) {
    return cast_to()->run(std::forward<Args>(args)...);
  }

  void allocSpace(size_t size) { return cast_to()->alloc(size); }

  template <typename... Args>
    requires(std::tuple_size_v<ParamTuple> + sizeof...(Args) <=
             Meta::traits<decltype(&Derived::eval)>::param_size)
  auto operator()(Args &&...args) {
    using p_type = decltype(std::tuple_cat(
        _args, std::make_tuple(std::forward<Args>(args)...)));
    return CrtpExprBase<device, Derived, p_type>(
        std::tuple_cat(_args, std::make_tuple(std::forward<Args>(args)...)));
  }

  // template <typename... Args>
  // requires {
  //   std::tuple_size_v<ParamTuple> + sizeof...(Args) ==
  //       Meta::traits<decltype(&Derived::eval)>::param_size
  // } auto operator()(Args &&...args) {
  //   return Meta::tuple_apply(cast_to()->eval, _args);
  // }

  template <typename Rhs_Derived>
  auto operator|(CrtpExprBase<device, Rhs_Derived> &&exp) {}
};

template <Device device, typename Alloctor, typename EleDataType>
class OpBase
    : public CrtpExprBase<device, OpBase<device, Alloctor, EleDataType>> {
public:
  int eval(int a) { return a; }
};
} // namespace Hexer