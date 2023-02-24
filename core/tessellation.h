#include "expr.h"

#include <cinolib/meshes/meshes.h>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
namespace Hexer {
// return connected and oppsite vertices of an edge, first 2 elements of the
// return array are vertices connected to the edge, the others are oppsite
// vertices
template <typename M, typename V, typename E, typename P>
auto edgeSample_Loop(const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                     int eid, double directCntWg, double oppWg) {
  cinolib::vec3d pos = directCntWg * (mesh.vert_data(mesh.edge_data(eid)[0]) +
                                      mesh.vert_data(mesh.edge_data(eid)[1]));
  for (const auto &fid : mesh.adj_e2p(eid)) {
    for (const auto &vid : mesh.poly_data(fid)) {
      if (vid != opps[0] && vid != opps[1])
        pos += oppWg * mesh.vert_data(vid);
    }
  }
  return pos;
}

// auxiliary class using for generating new vertices for Loop auxiliary
// algorithm
template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LoopAux_addVertex
    : public CrtpExprBase<device, LoopAux_addVertex, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(const cinolib::AbstractPolygonMesh<M, V, E, P> &ori_mesh) {
    std::vector<int> edges_map(ori_mesh.num_edges(), -1);
    std::vector<cinolib::vec3d> new_vertex;
    new_vertex.reserve(ori_mesh.num_edges() * 3);
    std::vector<cinolib::vec3u> new_polys;
    new_polys.reserve(4 * ori_mesh.num_polys());

    for (int fid = 0; fid < ori_mesh.num_polys(); ++fid) {
      uint new_vertex[3];
      for (const auto &[index, eid] :
           ori_mesh.adj_p2e(fid) | ranges::view::enumerate) {
        if (edges_map[eid] == -1) {
          cinolib::vec3d pos = edgeSample_Loop(ori_mesh, eid, 0.375, 0.125);
          new_vertex.push_back(pos);
          edges_map[eid] = ori_mesh.num_verts() + new_vertex.size() - 1;
        }
        new_vertex[index] = edges_map[eid];
      }

      // using new vertices to form a poly
      new_polys.push_back({new_vertex[0], new_vertex[1], new_vertex[2]});
      // connect new vertices to original vertices
      for (const auto &[i, vid] :
           ori_mesh.poly_data(fid) | ranges::view::enumerate)
        new_polys.push_back({vid, new_vertex[i], new_vertex[i % 3 - 1]});
    };
    return cinolib::Polygonmesh<M, V, E, P>(
        ranges::view::concat(ori_mesh.vector_verts(), new_vertex), new_polys);
  }
};

template <typename M, typename V, typename E, typename P>
auto vertexSample_Loop(const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                       int vid) {
  int count = mesh.adj_v2e(vid);
  ranges:(mesh.adj_v2v(vid) | ranges::view::all);
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LoopAux_adjustVertex
    : public CrtpExprBase<device, LoopAux_adjustVertex, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(const cinolib::AbstractPolygonMesh<M, V, E, P> &ori_mesh) {}
};

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