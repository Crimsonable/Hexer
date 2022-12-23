#pragma once
#include "expr.h"

#include <Eigen/SparseCore>
#include <vector>
// #include <OpenVolumeMesh/Core/Iterators/FaceIter.hh>
// #include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <cinolib/meshes/meshes.h>
#include <cinolib/smoother.h>

namespace Hexer {

class Convert2SurfaceMesh
    : public CrtpExprBase<Device::CPU, Convert2SurfaceMesh> {
public:
  template <typename M, typename V, typename E, typename F, typename P1,
            typename P2>
  static void
  eval(cinolib::AbstractPolygonMesh<M, V, E, P1> &surface,
       const cinolib::AbstractPolyhedralMesh<M, V, E, F, P2> &mesh) {
    std::vector<cinolib::vec3d> vertices_coords;
    std::vector<std::vector<uint>> faces;
    std::map<uint, uint> vertices_map;

    for (uint fid = 0; fid < mesh.num_faces(); ++fid) {
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

    surface.init(vertices_coords, faces);
  }
};

} // namespace Hexer