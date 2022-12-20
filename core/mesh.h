#pragma once
#include "expr.h"

#include <Eigen/SparseCore>
#include <vector>
// #include <OpenVolumeMesh/Core/Iterators/FaceIter.hh>
// #include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <cinolib/meshes/meshes.h>
#include <cinolib/smoother.h>

namespace Hexer {

template <typename M, typename V, typename E, typename P>
double uniform_weighter(std::vector<std::pair<uint, double>> &weights, uint vid,
                        const cinolib::AbstractMesh<M, V, E, P> &mesh) {
  for (uint vvid : mesh.adj_v2v(vid))
    weights.push_back({vvid, 1.0});
}

class MeshAdjacencyMatrix
    : public CrtpExprBase<Device::CPU, MeshAdjacencyMatrix> {
public:
  template <typename Weighter, typename M, typename V, typename E, typename P>
  static auto eval(Weighter &&weighter,
                   const cinolib::AbstractMesh<M, V, E, P> &mesh) {
                    cinolib::AbstractPolygonMesh<M,V,E,P> m;
                    m.mesh_is_volumetric();
    auto adjacency =
        Eigen::SparseMatrix<double>(mesh.num_verts(), mesh.num_verts());
    for (uint vid = 0; vid < mesh.num_verts(); ++vid) {
      std::vector<std::pair<uint, double>> weights;
      weighter(weights, vid, mesh);
      double sum = 0.0;
      for (auto &&w : weights) {
        sum += w.second;
        adjacency.insert(vid, w.first) = -w.second;
      }
      adjacency.insert(vid, vid) = sum;
    }
    adjacency.makeCompressed();
    return adjacency;
  }
};

} // namespace Hexer