#pragma once
#include "expr.h"

#include <OpenVolumeMesh/Core/Iterators/FaceIter.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <vector>

namespace Hexer {
using FaceIter = OpenVolumeMesh::FaceHandle;

class MeshFilter : public CrtpExprBase<Device::CPU, MeshFilter> {
  static auto eval(MeshType meshtype, PolyhedralMesh &mesh);
};
} // namespace Hexer