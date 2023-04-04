#pragma once
#include "expr.h"

#include <Eigen/SparseCore>
#include <cinolib/meshes/meshes.h>
#include <vector>

namespace Hexer {

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class Convert2SurfaceMesh
    : public CrtpExprBase<device, Convert2SurfaceMesh<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh) {
    std::vector<cinolib::vec3d> vertices_coords;
    std::vector<std::vector<uint>> faces;
    std::map<uint, uint> vertices_map;

    for (int fid = 0; fid < mesh.num_faces(); ++fid) {
      if (mesh.face_is_on_srf(fid)) {
        auto vids = mesh.face_verts_id(fid);
        faces.emplace_back();
        auto &current_face = faces.back();
        for (auto &&vid : vids) {
          if (vertices_map.find(vid) == vertices_map.end()) {
            vertices_coords.push_back(mesh.vert(vid));
            vertices_map[vid] = vertices_coords.size() - 1;
          }
          current_face.push_back(vertices_map[vid]);
        }
      }
    }

    cinolib::Polygonmesh<> surface;
    surface.init(vertices_coords, faces);
    return surface;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class PolyhedralSurfMarker
    : public CrtpExprBase<device, PolyhedralSurfMarker<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh) {
    for (auto &fid : mesh.get_surface_faces()) {
      mesh.face_data(fid).flags[cinolib::UNUSED_0] = 1;
      mesh.poly_data(mesh.adj_f2p(fid)[0]).flags[cinolib::UNUSED_0] = 1;
    }
  }
};

} // namespace Hexer