#include <iostream>
#include <vector>

#include "core/expr.h"
#include "core/filter.h"
#include "core/io.h"
#include "core/deformation.h"

int main() {
  Hexer::PolyMeshReader reader;
  //auto filter = Hexer::MeshTypeFilter()(Hexer::MeshType::Quads);
  auto mesh = reader.execute("D:/codes/Hexer/models/s01c_cube.vtk");
  //auto triangle_set = filter.execute(mesh);
  return 1;
}
