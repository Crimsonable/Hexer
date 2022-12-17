#pragma once
#include "expr.h"

#include "mesh.h"
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <vector>

namespace Hexer {
using FaceIter = OpenVolumeMesh::FaceHandle;

class MeshFilter : public CrtpExprBase<Device::CPU, MeshFilter> {
  static auto eval(MeshType meshtype, PolyhedralMesh &out_mesh,
                   std::map<unsigned int, unsigned int> &map,
                   const OpenVolumeMesh::FaceHandle &face,
                   PolyhedralMesh *mesh);
};

} // namespace Hexer