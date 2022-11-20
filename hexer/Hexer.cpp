// #include <OpenMesh/Core/IO/BinaryHelper.hh>
#include <iostream>

#include "Hexer.h"
// #include "core/op.h"
// #include "core/ops.h"
#include "core/expr.h"
#include <vector>

int main() { 
  Hexer::OpBase<Hexer::Device::CPU, int, int> op;
  int a=1;
  auto op2=op(a);
  return 1;
 }
