#pragma once
#include "expr.h"
#include "mesh.h"

namespace Hexer {
    class LaplacianSmoother:public CrtpExprBase<Device::CPU,LaplacianSmoother>{
        public:
        static void eval(const OpenVolumeMesh::VertexHandle& vertex,PolyhedralMesh* mesh);
    };
}