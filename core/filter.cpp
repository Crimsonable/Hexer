#include "filter.h"

#include <Expblas/ftensor.h>
#include <map>

using namespace Hexer;

auto MeshFilter::eval(MeshType meshtype, PolyhedralMesh &out_mesh,
                      std::map<unsigned int, unsigned int> &map,
                      const OpenVolumeMesh::FaceHandle &face,
                      PolyhedralMesh *mesh) {
  if (mesh->face(face).halfedges().size() == int(meshtype)) {
    std::vector<OpenVolumeMesh::VertexHandle> face_vertices;
    for (auto &&edge_iter : mesh->face(face).halfedges()) {
      auto v_index = mesh->halfedge(edge_iter).from_vertex();
      if (map.find(v_index.uidx()) == map.end())
        map[v_index.uidx()] = out_mesh.add_vertex(mesh->vertex(v_index)).uidx();
      face_vertices.emplace_back(map[v_index.uidx()]);
    }
    out_mesh.add_face(face_vertices);
  }
}