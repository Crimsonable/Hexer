#pragma once
#include "expr.h"

#include <cinolib/meshes/meshes.h>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

namespace Hexer {
// return connected and oppsite vertices of an edge, first 2 elements of the
// return array are vertices connected to the edge, the others are oppsite
// vertices
template <typename M, typename V, typename E, typename P>
auto edgeSample_Loop(const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                     int eid, double directCntWg, double oppWg) {
  uint id0 = mesh.edge_vert_ids(eid)[0];
  uint id1 = mesh.edge_vert_ids(eid)[1];
  cinolib::vec3d pos = directCntWg * (mesh.vert(id0) + mesh.vert(id1));

  for (const auto &fid : mesh.adj_e2p(eid)) {
    for (const auto &vid : mesh.poly_verts_id(fid))
      if (vid != id0 && vid != id1)
        pos += oppWg * mesh.vert(vid);
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
    new_vertex.reserve(ori_mesh.num_edges());
    std::vector<cinolib::vec3u> new_polys;
    new_polys.reserve(4 * ori_mesh.num_polys());

    for (int fid = 0; fid < ori_mesh.num_polys(); ++fid) {
      uint new_vertex_tmp[3];
      for (const auto &[index, eid] :
           ori_mesh.adj_p2e(fid) | ranges::view::enumerate) {
        if (edges_map[eid] == -1) {
          cinolib::vec3d pos = edgeSample_Loop(ori_mesh, eid, 0.375, 0.125);
          new_vertex.push_back(pos);
          edges_map[eid] = ori_mesh.num_verts() + new_vertex.size() - 1;
        }
        new_vertex_tmp[index] = edges_map[eid];
      }

      // using new vertices to form a poly
      new_polys.push_back(
          {new_vertex_tmp[0], new_vertex_tmp[1], new_vertex_tmp[2]});
      // connect new vertices to original vertices
      for (const auto &[i, vid] : ori_mesh.poly_verts_id_const(fid) |
                                      ranges::views::all |
                                      ranges::view::enumerate)
        new_polys.push_back(
            {vid, new_vertex_tmp[i], new_vertex_tmp[(i + 2) % 3]});
    };
    return cinolib::Polygonmesh<M, V, E, P>(
        ranges::view::concat(ori_mesh.vector_verts(), new_vertex),
        new_polys | ranges::view::transform([](const auto &x) {
          return std::vector<uint>({x[0], x[1], x[2]});
        }));
  }
};

template <typename M, typename V, typename E, typename P>
auto vertexSample_Loop(const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                       int vid) {
  int count = mesh.adj_v2e(vid);
  double alpha = 0.625 - std::pow(0.375 + 0.25 * std::cos(2 * M_PI / count), 2);
  auto view = mesh.adj_v2v(vid) | ranges::view::transform([](const auto &id) {
                return mesh.vert(id);
              });
  return 0.5 * alpha / count *
             ranges::accumulate(view, cinolib::vec3d(0, 0, 0)) +
         (0.5 - alpha) * mesh.vert(vid);
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LoopAux_adjustVertex
    : public CrtpExprBase<device, LoopAux_adjustVertex, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(cinolib::AbstractPolygonMesh<M, V, E, P> &&mesh, int n) {
    for (const auto &vid : ranges::view::iota(n))
      mesh.adj_v2v(vid) | ranges::view::transform([&](const auto &i) {
        mesh.vert(i) = vertexSample_Loop(mesh, i);
      });
    return mesh;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LoopSubdivision
    : public CrtpExprBase<device, LoopSubdivision, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(const cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
    return (LoopAux_addVertex()(mesh) | LoopAux_adjustVertex()).execute();
  }
};
} // namespace Hexer