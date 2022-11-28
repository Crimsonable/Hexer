#pragma once
#include "expr.h"

#include <OpenVolumeMesh/Core/Iterators/FaceIter.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <vector>

namespace Hexer {
using FaceIter = OpenVolumeMesh::FaceHandle;

class MeshTypeFilter : public CrtpExprBase<Device::CPU, MeshTypeFilter> {
public:
  static std::vector<FaceIter> eval(MeshType type, const PolyhedralMesh &mesh);
};
} // namespace Hexer