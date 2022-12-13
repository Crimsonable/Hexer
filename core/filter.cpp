#include "filter.h"

#include <Expblas/ftensor.h>
#include <map>

using namespace Hexer;

auto MeshFilter::eval(MeshType meshtype, PolyhedralMesh &mesh) {
  PolyhedralMesh tri_mesh;
  std::map<unsigned int, unsigned int> vertex_map;
  for (auto &&iter : mesh.faces()) {
    if (mesh.face(iter).halfedges().size() == int(meshtype)) {
      std::vector<OpenVolumeMesh::VertexHandle> face_vertices;
      for (auto &&edge_iter : mesh.face(iter).halfedges()) {
        auto v_index = mesh.halfedge(edge_iter).from_vertex();
        if (vertex_map.find(v_index.uidx()) == vertex_map.end())
          vertex_map[v_index.uidx()] =
              tri_mesh.add_vertex(mesh.vertex(v_index)).uidx();
        face_vertices.emplace_back(vertex_map[v_index.uidx()]);
      }
      tri_mesh.add_face(face_vertices);
    }
  }
  return tri_mesh;
}