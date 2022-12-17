#pragma once
#include "expr.h"

#include <OpenVolumeMesh/Core/Iterators/FaceIter.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

namespace Hexer {

class PolyhedralMesh : public OpenVolumeMesh::GeometricPolyhedralMeshV3f {
public:
  PolyhedralMesh() : OpenVolumeMesh::GeometricPolyhedralMeshV3f() {}

  template <typename Op> auto faces_apply(Op &&op) {
    for (auto &&iter : this->faces())
      op.execute(std::forward<decltype(iter)>(iter), this);
  }

  template <typename Op> auto edges_apply(Op &&op) {
    for (auto &&iter : this->edges())
      op.execute(std::forward<decltype(iter)>(iter), this);
  }

  template <typename Op> auto vertices_apply(Op &&op) {
    for (auto &&iter : this->vertices())
      op.execute(std::forward<decltype(iter)>(iter), this);
  }
};

} // namespace Hexer