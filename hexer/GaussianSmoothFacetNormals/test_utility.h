#pragma once
#include <Expblas/ftensor.h>
#include <cinolib/dual_mesh.h>
#include <cinolib/meshes/meshes.h>

void projectUnitSphere(cinolib::Polygonmesh<> &mesh) {
  for (auto &v : mesh.vector_verts())
    v.normalize();
}

auto unitTetrahedron() {
  cinolib::Polygonmesh<> mesh;
  double a = 1.0 / 3.0;
  double b = std::sqrt(8.0 / 9.0);
  double c = std::sqrt(2.0 / 9.0);
  double d = std::sqrt(2.0 / 3.0);

  mesh.vert_add({0, 0, 1});
  mesh.vert_add({-c, d, -a});
  mesh.vert_add({-c, -d, -a});
  mesh.vert_add({b, 0, -a});

  mesh.poly_add({0, 1, 2});
  mesh.poly_add({0, 2, 3});
  mesh.poly_add({0, 3, 1});
  mesh.poly_add({3, 2, 1});

  return mesh;
}

auto unitHexahedron() {
  cinolib::Polygonmesh<> mesh;
  double l = 1.0 / std::sqrt(3.0);
  mesh.vert_add({l, l, l});    // 0
  mesh.vert_add({-l, l, l});   // 1
  mesh.vert_add({l, -l, l});   // 2
  mesh.vert_add({l, l, -l});   // 3
  mesh.vert_add({-l, -l, l});  // 4
  mesh.vert_add({-l, l, -l});  // 5
  mesh.vert_add({l, -l, -l});  // 6
  mesh.vert_add({-l, -l, -l}); // 7

  mesh.poly_add({0, 3, 5, 1});
  mesh.poly_add({0, 1, 4, 2});
  mesh.poly_add({0, 2, 6, 3});
  mesh.poly_add({7, 4, 1, 5});
  mesh.poly_add({7, 6, 2, 4});
  mesh.poly_add({7, 5, 3, 6});

  return mesh;
}

auto unitOctahedron() {
  cinolib::Polygonmesh<> mesh;
  cinolib::dual_mesh(unitHexahedron(), mesh, true);
  projectUnitSphere(mesh);
  return mesh;
}

cinolib::DrawableTrimesh<>
SphereGen(int n, const std::vector<Expblas::vec3d> &vertex,
          const std::vector<Expblas::vec3ui> &faces) {
  std::vector<Expblas::vec3d> _vertex = vertex;
  std::vector<Expblas::vec3ui> _faces = faces;
  _faces.reserve(faces.size() * n * 3);

  for (int i = 0; i < n; ++i) {
    uint count = _faces.size();
    std::vector<Expblas::vec3ui> new_faces;
    new_faces.reserve(3 * count);
    for (uint j = 0; j < count; ++j) {
      auto &f = faces[j];
      for (int k = 0; k < 3; ++k) {
        _vertex.push_back(Expblas::normalized(
            0.5 * (_vertex[f[k]] + _vertex[f[(k + 1) % 3]])));
        // new_faces.push_back({_vertex.size() - 1, f[k], f[(k + 1) % 3]});
      }
      uint s = _vertex.size();
      new_faces.push_back({s - 3, s - 2, s - 1});
      new_faces.push_back({s - 3, s - 1, _faces.back()[0]});
      new_faces.push_back({s - 2, s - 3, _faces.back()[1]});
      new_faces.push_back({s - 1, s - 2, _faces.back()[2]});
      //_faces.pop_back();
    }
    std::ranges::move(new_faces, std::back_inserter(_faces));
  }

  cinolib::DrawableTrimesh<> mesh(
      std::vector<double>(_vertex.data()->dataptr(),
                          _vertex.data()->dataptr() + 3 * _vertex.size()),
      std::vector<uint>(_faces.data()->dataptr(),
                        _faces.data()->dataptr() + _faces.size() * 3));
  return mesh;
}

template <typename Mesh> class NewCanvans : public cinolib::GLcanvas {
  int count = 0;

public:
  void draw() override {
    if (count % 100 == 0) {
      for (auto obj : drawlist)
        const_cast<cinolib::AbstractDrawablePolyhedralMesh<Mesh> *>(
            dynamic_cast<const cinolib::AbstractDrawablePolyhedralMesh<Mesh> *>(
                obj))
            ->updateGL();
      count = 0;
    }
    static_cast<cinolib::GLcanvas *>(this)->draw();
    count++;
  }
};