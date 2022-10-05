#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "graph.h"

namespace Hexer {
enum class MeshType { Triangle, Hex };
using TriMeshData = OpenMesh::TriMesh_ArrayKernelT<>;
using HexMeshData = OpenMesh::PolyMesh_ArrayKernelT<>;

template <typename MeshType> struct MeshData;

template <> class MeshData<TriMeshData> : public GraphSlot {
public:
  TriMeshData _data;
};

template <> class MeshData<HexMeshData> : public GraphSlot {
public:
  HexMeshData _data;
}

template <MeshType Type>
class MeshIO;

template <> class MeshIO<MeshType::Triangle> : public GraphVertex {
public:
  MeshIO() { outputs.push_back(&_mesh); }

  bool readMesh(const std::string &path) {
    return OpenMesh::IO::read_mesh(_mesh, path);
  }

  size_t edgesCount() const { return _mesh._data.n_edges(); }
  size_t facesCount() const { return _mesh._data.n_faces(); }

private:
  MeshData<TriMeshData> _mesh;
}
} // namespace Hexer