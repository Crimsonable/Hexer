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

class dummyclass : public CrtpExprBase<Device::CPU, dummyclass> {
public:
  template <typename T> static auto eval(T a) { return 1; }
};

template <typename Weighter, typename M = cinolib::Mesh_std_attributes,
          typename V = cinolib::Vert_std_attributes,
          typename E = cinolib::Edge_std_attributes,
          typename P = cinolib::Polygon_std_attributes>
class LaplacianMatrix
    : public CrtpExprBase<Device::CPU, LaplacianMatrix<Weighter, M, V, E, P>> {
public:
  static auto eval(Weighter &&weighter,
                   const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    auto Laplacian =
        Eigen::SparseMatrix<double>(mesh.num_verts(), mesh.num_verts());
    for (uint vid = 0; vid < mesh.num_verts(); ++vid) {
      std::vector<std::pair<uint, double>> weights;
      weighter(weights, vid, mesh);
      double sum = 0.0;
      for (auto &&w : weights) {
        sum += w.second;
        Laplacian.insert(vid, w.first) = -w.second;
      }
      Laplacian.insert(vid, vid) = sum;
    }
    Laplacian.makeCompressed();
    return Laplacian;
  }
};

} // namespace Hexer