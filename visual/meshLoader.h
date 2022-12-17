#pragma once
#include "core/filter.h"
#include "model.h"

namespace Visual {
Model convertMeshToModel(Hexer::PolyhedralMesh &mesh);

class MeshToModelConverter
    : public Hexer::CrtpExprBase<Hexer::Device::CPU, MeshToModelConverter> {
public:
  template <typename FaceFilter>
  static std::vector<Mesh> eval(FaceFilter &&filter,
                                Hexer::PolyhedralMesh &mesh) {}
};

class MeshFilterVisual
    : public Hexer::CrtpExprBase<Hexer::Device::CPU, MeshFilterVisual> {
public:
  static void eval(Hexer::MeshType meshtype, Mesh &out_mesh,
                   std::map<unsigned int, unsigned int> &map,
                   const OpenVolumeMesh::FaceHandle &face,
                   Hexer::PolyhedralMesh *mesh);
};

} // namespace Visual