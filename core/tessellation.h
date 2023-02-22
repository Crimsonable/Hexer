#include "expr.h"

namespace Hexer {
template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LoopSubdivision
    : public CrtpExprBase<device, LoopSubdivision, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
    cinolib::Polygonmesh<M, V, E, P> res_mesh;
    for (int eid = 0; eid < mesh.num_edges(); ++eid) {
      std::vector<uint> vadj = mesh.adj_e2v(eid);
      std::vector<uint> vopp;
      vopp.for (const auto &fid : mesh.adj_e2p()) {
        for (const auto &vid : mesh.poly_data(fid))
          if (std::find(vadj.begin(), vadj.end(), vid) == vadj.end()) {
            vopp.push_back(vid);
            break;
          }
      }
      cinolib::vec3d pos(0, 0, 0);
      for (const auto &vid : vadj)
        pos = pos + 0.375 * mesh.vert_data(vid);
      for (const auto &vid : vopp)
        pos = pos + 0.125 * mesh.vert_data(vid);

      res_mesh.vert_add(pos);
    }
    auto weighter = [](int n) {
      return 0.625 - std::pow(0.375 + 0.25 * std::cos(2 * M_PI / n), 2);
    };
    for ()
  }
};
} // namespace Hexer