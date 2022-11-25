// #include <OpenMesh/Core/IO/BinaryHelper.hh>
#include <iostream>

#include "Hexer.h"
// #include "core/op.h"
// #include "core/ops.h"
#include "core/expr.h"
#include <vector>

int main() {
  Hexer::OpBase<Hexer::Device::CPU, int, int> op;
  Hexer::OpBase<Hexer::Device::CPU, int, double> op2;
  int a = 1;
  auto opx = op(1) | op2(2);
  opx.execute(2);
  return 1;
}
