#include "meshLoader.h"

void Visual::convertMeshToModel(Hexer::PolyhedralMesh &mesh, Model &model) {
  Hexer::MeshTypeFilter tri_filter;
  auto indice = tri_filter(Hexer::MeshType::Triangle).execute(mesh);
  std::vector<Vertex> vertices;
  for (auto &&iter : mesh.vertex_positions()) {
    Vertex vertex;
    vertex.Position = *reinterpret_cast<Expblas::vec3f *>(&iter);
    vertices.push_back(vertex);
  }
    
}