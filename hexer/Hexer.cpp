// #include <OpenMesh/Core/IO/BinaryHelper.hh>
#include <iostream>

#include "Hexer.h"
// #include "core/op.h"
// #include "core/ops.h"
#include "core/expr.h"
#include <vector>

int main() { 
  Hexer::OpBase<Hexer::Device::CPU, int, int> op;
  auto op2=op(1);
  return 1;
 }
