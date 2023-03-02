#pragma once
#include "base.h"
#include "traits.h"

#include <tuple>

namespace Hexer {
template <typename Derived, typename Expr, Device device> class ExprBaseCrtp;

template <typename Derived, Device device>
class ExprBaseCrtp<Derived, void, device>
    : public ExprBaseCrtp<Derived, void, device> {
public:
  ExprBaseCrtp() {}

  template <typename Expr> auto operator()(const SubExpr &expr) {
    return ExprBaseCrtp<Derived, Expr, device>(*this, expr);
  }
};

template <typename Derived, typename Expr, Device device>
class ExprBaseCrtp : public ExprBaseCrtp<Derived, Expr, device> {
  Expr &&_expr;
  Derived &&_self;

public:
  template <typename SubExpr>
  ExprBaseCrtp(Derived &&self, SubExpr &&expr) : _self(self), _expr(expr) {}

  template <typename SubExpr> auto operator()(const SubExpr &expr) {
    return ExprBaseCrtp<ExprBaseCrtp<Derived, Expr, device>, SubExpr, device>(
        *this, expr);
  }
};

class op : public ExprBaseCrtp<op, void, Device::CPU> {};

void f() { auto a = op(op()); }

} // namespace Hexer