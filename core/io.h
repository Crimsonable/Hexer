#pragma once
#include "expr.h"
#include <string>

#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>

namespace Hexer {
using PolyhedralMesh = OpenVolumeMesh::GeometricPolyhedralMeshV3f;

class PolyMeshReader : public CrtpExprBase<Device::CPU, PolyMeshReader> {
public:
  static PolyhedralMesh eval(const std::string& path);
};
} // namespace Hexer