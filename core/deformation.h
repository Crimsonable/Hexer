#pragma once
#include "expr.h"
#include "mesh.h"

namespace Hexer {
class UniformWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<std::pair<uint, double>> &weights, uint vid,
                  const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    for (uint vvid : mesh.adj_v2v(vid))
      weights.push_back({vvid, 1.0});
  }
};

class LaplacianMatrix : public CrtpExprBase<Device::CPU, LaplacianMatrix> {
public:
  template <typename Weighter, typename M, typename V, typename E, typename P>
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

class LaplacianSmoother:public CrtpExprBase<Device::CPU,LaplacianSmoother>{
    public:
    template<typename M,typename V,typename E,typename P>
    static auto eval()
};
} // namespace Hexer