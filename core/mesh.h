#pragma once
#include "expr.h"

#include <Eigen/SparseCore>
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

class MeshAdjacencyMatrix
    : public CrtpExprBase<Device::CPU, MeshAdjacencyMatrix> {
public:
  static auto eval(Eigen::SparseMatrix<float> &adjacency,
                   PolyhedralMesh *surf_mesh) {
    int n = surf_mesh->n_vertices();
    adjacency = Eigen::SparseMatrix<float>(n, n);
    for (auto &&iter : surf_mesh->edges()) {
      auto start_idx = surf_mesh->edge(iter).from_vertex().uidx();
      auto end_idx = surf_mesh->edge(iter).to_vertex().uidx();
      adjacency.insert(start_idx, end_idx) = 1;
    }
  }
};

} // namespace Hexer