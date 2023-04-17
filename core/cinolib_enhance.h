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

template <typename M, typename V, typename E, typename F, typename P>
auto vert_offset_within_tri(
    const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh, int fid,
    int vid) {
  for (int i = 0; i < 3; ++i)
    if (mesh.face_vert_id(fid, i) == vid)
      return i;
  return -1;
}
} // namespace Hexer