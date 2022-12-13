#pragma once
#include "core/filter.h"
#include "model.h"

namespace Visual {
void convertMeshToModel(Hexer::PolyhedralMesh &mesh, Model &model);

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
  static std::pair<std::vector<Vertex>, std::vector<unsigned int>>
  eval(Hexer::MeshType meshtype, Hexer::PolyhedralMesh &mesh);
};

} // namespace Visual