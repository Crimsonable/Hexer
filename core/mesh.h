#pragma once
#include "expr.h"

#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

namespace Hexer {
template <typename Handle>
concept CFaceHandle = std::is_same_v<OpenVolumeMesh::FaceHandle, Handle>;

template <typename Handle>
concept CEdgeHandle = std::is_same_v<OpenVolumeMesh::EdgeHandle, Handle>;

class PolyMeshContainer : public Stream<Device::CPU, PolyhedralMesh> {
public:
  PolyMeshContainer();

  template <CFaceHandle HandleType>
  
};
} // namespace Hexer