#pragma once
#include "core/filter.h"
#include "model.h"

namespace Visual {
void convertMeshToModel(Hexer::PolyhedralMesh &mesh, Model &model);

class MeshToModelConverter
    : public Hexer::CrtpExprBase<Hexer::Device::CPU, MeshToModelConverter> {
        public:

        template<typename FaceFilter>
        static std::vector<Mesh> eval(FaceFilter&& filter,Hexer::PolyhedralMesh& mesh){

        }
    };

} // namespace Visual