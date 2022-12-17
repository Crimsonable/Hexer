#pragma once
#define HEXER_LOG
#define HEXER_INLINE __forceinline

#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

namespace Hexer {
using uint = unsigned int;
using ID = size_t;
//using PolyhedralMesh = OpenVolumeMesh::GeometricPolyhedralMeshV3f;

enum class Device { CPU, GPU };
enum class MeshType { Triangle = 3, Quads = 4 };

template <Device device, typename T> struct MemoryManager;

template <Device device, typename Derived, typename ParamTuple = std::tuple<>>
class CrtpExprBase;
template <Device device, typename Alloctor, typename EleDataType> class OpBase;

} // namespace Hexer