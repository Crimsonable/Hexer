#pragma once

#include "base.h"
#include "expr.h"

namespace Hexer {
template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class Cycle
    : public CrtpExprBase<device, Cycle<device, ParamTuple>, ParamTuple> {
public:


  template <Meta::ConceptExpr Expr, typename... Args>
  auto eval(Expr &&expr, int n, Args &&..args) {

  }
};
} // namespace Hexer