#include <iostream>
#include <vector>

#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

namespace Hexer {
using Vec3f = OpenVolumeMesh::Geometry::Vec3f;

struct Polyhedral_Mesh {
  OpenVolumeMesh::GeometricPolyhedralMeshV3f _mesh;
};
} // namespace Hexer
