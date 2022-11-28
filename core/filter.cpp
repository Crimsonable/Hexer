#include "filter.h"

std::vector<Hexer::FaceIter>
Hexer::MeshTypeFilter::eval(MeshType type, const PolyhedralMesh &mesh) {
  std::vector<FaceIter> filter_set;
  for (auto &&iter : mesh.faces())
    if (mesh.face(iter).halfedges().size() == size_t(type))
      filter_set.push_back(iter);
  return filter_set;
}