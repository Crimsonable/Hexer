#include <cinolib/meshes/meshes.h>

namespace Hexer {
template <typename M, typename V, typename E, typename F, typename P>
auto vert_offset_within_tet(
    const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh, int pid,
    int vid) {
  for (int i = 0; i < 4; ++i)
    if (mesh.poly_vert_id(pid, i) == vid)
      return i;
  return -1;
}
} // namespace Hexer