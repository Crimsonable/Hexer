#include "meshLoader.h"

void Visual::convertMeshToModel(Hexer::PolyhedralMesh &mesh, Model &model) {
  Mesh &visual_mesh = model.addMesh();

  auto filter =
      Visual::MeshFilterVisual().operator()(Hexer::MeshType::Triangle);
  auto [vertices, indices] = filter.execute(mesh);
  visual_mesh.vertices = vertices;
  visual_mesh.indices = indices;
}

std::pair<std::vector<Visual::Vertex>, std::vector<unsigned int>>
Visual::MeshFilterVisual::eval(Hexer::MeshType meshtype,
                               Hexer::PolyhedralMesh &mesh) {
  std::map<unsigned int, unsigned int> vertex_map;
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  for (auto &&iter : mesh.faces()) {
    if (mesh.face(iter).halfedges().size() == int(meshtype)) {
      for (auto &&edge_iter : mesh.face(iter).halfedges()) {
        auto v_index = mesh.halfedge(edge_iter).from_vertex();
        if (vertex_map.find(v_index.uidx()) == vertex_map.end()) {
          auto &v = mesh.vertex(v_index);
          vertices.push_back(Vertex{Expblas::vec3f(v[0], v[1], v[2])});
          vertex_map[v_index.uidx()] = vertices.size() - 1;
        }
        indices.push_back(vertex_map[v_index.uidx()]);
      }
    }
  }
  return std::make_pair(std::move(vertices), std::move(indices));
}