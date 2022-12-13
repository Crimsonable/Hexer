#include "filter.h"

bool Hexer::MeshTypeFilter::eval(const MeshType &type,
                                 const OpenVolumeMesh::FaceHandle &face,
                                 const PolyhedralMesh &mesh) {
  return mesh.face(face).halfedges().size() == size_t(type);
}