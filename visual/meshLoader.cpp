#include "meshLoader.h"

Visual::Model Visual::convertMeshToModel(Hexer::PolyhedralMesh &mesh) {
  Visual::Model model;
  Mesh &visual_mesh = model.addMesh();
  std::map<unsigned int, unsigned int> v_map;

  auto filter = Visual::MeshFilterVisual().operator()(Hexer::MeshType::Triangle,
                                                      visual_mesh, v_map);
  mesh.faces_apply(filter);
  visual_mesh.setupMesh();
  return model;
}

void Visual::MeshFilterVisual::eval(Hexer::MeshType meshtype, Mesh &out_mesh,
                                    std::map<unsigned int, unsigned int> &map,
                                    const OpenVolumeMesh::FaceHandle &face,
                                    Hexer::PolyhedralMesh *mesh) {
  std::vector<Vertex> &vertices = out_mesh.vertices;
  std::vector<unsigned int> &indices = out_mesh.indices;

  if (mesh->face(face).halfedges().size() == int(meshtype)) {
    for (auto &&edge_iter : mesh->face(face).halfedges()) {
      auto v_index = mesh->halfedge(edge_iter).from_vertex();
      if (map.find(v_index.uidx()) == map.end()) {
        auto &v = mesh->vertex(v_index);
        vertices.push_back(Vertex{Expblas::vec3f(v[0], v[1], v[2])});
        map[v_index.uidx()] = vertices.size() - 1;
      }
      indices.push_back(map[v_index.uidx()]);
    }
  }
}