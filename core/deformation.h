#pragma once
#include "expr.h"
#include "mesh.h"

namespace Hexer {
class UniformWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<Eigen::Triplet<double>> &entries, uint nv,
                  uint current_vid,
                  const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    auto ring = mesh.vert_n_ring(current_vid, 1);
    entries.emplace_back(current_vid, current_vid, ring.size());
    std::vector<uint> offset;

    for (int i = 0; i < nv; ++i)
      offset.push_back(i * mesh.num_verts());

    for (auto &&iter : ring)
      for (auto &&_offset : offset)
        entries.emplace_back(current_vid, iter + _offset, -1.0);
  }
};

class LaplacianMatrix : public CrtpExprBase<Device::CPU, LaplacianMatrix> {
public:
  template <typename Weighter, typename M, typename V, typename E, typename P>
  static auto eval(Weighter &&weighter,
                   const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    uint n = mesh.num_verts();
    auto Laplacian = Eigen::SparseMatrix<double>(n, n * 3);
    std::vector<Eigen::Triplet<double>> entries;
    for (uint vid = 0; vid < mesh.num_verts(); ++vid) {
      weighter(entries, 3, vid, mesh);
    }
    Laplacian.setFromTriplets(entries.begin(), entries.end());
    return Laplacian;
  }
};

class LaplacianSmoother : public CrtpExprBase<Device::CPU, LaplacianSmoother> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const Eigen::SparseMatrix<double> &Laplacian,
                   cinolib::AbstractPolygonMesh<M, V, E, P> &output,
                   const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
    std::vector<double> vertices_coords;
    uint n = mesh.num_verts();
    vertices_coords.resize(n * 3);
    output = mesh;

    for (int vid = 0; vid < n; ++vid) {
      auto p = mesh.vert(vid);
      vertices_coords[vid] = p.x();
      vertices_coords[vid + n] = p.y();
      vertices_coords[vid + 2 * n] = p.z();
    }
    Eigen::VectorXd v =
        Eigen::Map<Eigen::VectorXd>(vertices_coords.data(), n * 3);

    double lambda = 0.001;
    v -= lambda * Laplacian * v;
  }
};
} // namespace Hexer