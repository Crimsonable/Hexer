#include "base.h"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#ifdef HEXER_LOG
#include <spdlog/spdlog.h>
#endif

#include "graph.h"

namespace Hexer {
enum class MeshType { Triangle, Hex };
using TriMeshData = OpenMesh::TriMesh_ArrayKernelT<>;
using HexMeshData = OpenMesh::PolyMesh_ArrayKernelT<>;

template <typename MeshType> struct MeshData;

template <> class MeshData<TriMeshData> : public GraphEdge {
public:
  MeshData() : GraphEdge(GlobalID::getInstance()->getID()) {}
  TriMeshData _data;
};

template <> class MeshData<HexMeshData> : public GraphEdge {
public:
  MeshData() : GraphEdge(GlobalID::getInstance()->getID()) {}
  HexMeshData _data;
};

template <MeshType Type> class MeshIO;

template <> class MeshIO<MeshType::Triangle> : public GraphVertex {
public:
  Edge_list operator()(MeshData<TriMeshData> &mesh, const std::string &path) {
    _path = path;
    addInput(&mesh);
    return this->operator()({mesh});
  }

private:
  Edge_list eval() {
    TriMeshData &mesh_data =
        static_cast<MeshData<TriMeshData> *>(this->inwards[0])->_data;
    if (OpenMesh::IO::read_mesh(mesh_data, _path)) {
#ifdef HEXER_LOG
      auto console = spdlog::stdout_color_mt("console");
      auto err_logger = spdlog::stderr_color_mt("stderr");
      spdlog::get("console")->info(
          "loggers can be retrieved from a global registry using the "
          "spdlog::get(logger_name)");
#endif
      return {inwards[0]};
    }
    return {};
  }

  std::string _path;
};
} // namespace Hexer