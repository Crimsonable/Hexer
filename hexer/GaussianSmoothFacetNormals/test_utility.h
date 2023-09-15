#pragma once
#include <Expblas/ftensor.h>
#include <cinolib/dual_mesh.h>
#include <cinolib/meshes/meshes.h>
#include <range/v3/all.hpp>
#include <range/v3/view/linear_distribute.hpp>

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

template <typename M, typename V, typename E, typename F, typename P>
void meshInfoQuery(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh) {
  int count = 0;
  for (int vid = 0; vid < mesh.num_verts(); ++vid)
    if (!mesh.vert_is_on_srf(vid))
      count++;
  std::cout << "Verts inside: " << count << std::endl;

  count = 0;
  for (int fid = 0; fid < mesh.num_faces(); ++fid)
    if (!mesh.face_is_on_srf(fid))
      count++;
  std::cout << "Faces inside: " << count << std::endl;

  count = 0;
  for (int pid = 0; pid < mesh.num_polys(); ++pid)
    if (!mesh.poly_is_on_surf(pid))
      count++;
  std::cout << "Polys inside: " << count << std::endl;

  count = 0;
  uint f;
  for (int fid = 0; fid < mesh.num_faces(); ++fid)
    if (!mesh.face_is_visible(fid, f))
      count++;
  std::cout << "Face visible: " << count << std::endl;

  count = 0;
  for (int pid = 0; pid < mesh.num_polys(); ++pid)
    if (mesh.poly_data(pid).flags[2])
      count++;
  std::cout << "Hiden polys: " << count << std::endl;
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

class ColorMap {
  Eigen::Matrix3Xd _colormap = Eigen::Matrix3Xd({{235, 249, 118, 46, 235, 236},
                                                 {56, 252, 252, 34, 57, 51},
                                                 {35, 87, 122, 243, 183, 51}});
  Eigen::VectorXd _ranges;

  void init() {
    _ranges.resize(_colormap.cols() - 1);
    for (int i = 0; i < _colormap.cols() - 1; ++i)
      _ranges[i] = (_colormap.col(i + 1) - _colormap.col(i)).norm();

    for (int i = 1; i < _ranges.size(); ++i)
      _ranges[i] += _ranges[i - 1];
  }

public:
  ColorMap(const Eigen::Matrix3Xd &color_map) : _colormap(color_map) { init(); }

  ColorMap() { init(); }

  Eigen::Vector3d interp(double l) {
    for (int i = 0; i < _ranges.size(); ++i) {
      if (l > _ranges[i])
        continue;
      return i ? ((l - _ranges[i - 1]) / (_ranges[i] - _ranges[i - 1]) *
                      _colormap.col(i) +
                  (_ranges[i] - l) / (_ranges[i] - _ranges[i - 1]) *
                      _colormap.col(i + 1))
               : (l / _ranges[0] * _colormap.col(0) +
                  (_ranges[0] - l) / _ranges[0] * _colormap.col(1));
    }
  }

  std::vector<Eigen::Vector3d> interp(std::vector<double> l_list) {
    return l_list |
           ranges::views::transform([&](double l) { return this->interp(l); }) |
           ranges::to<std::vector>();
  }

  auto interp_n(int n) {
    return ranges::views::linear_distribute(double(0.0),
                                            double(*(_ranges.end() - 1)), n) |
           ranges::views::transform(
               [&](double l) { return (this->interp(l)).normalized(); }) |
           ranges::to<std::vector>();
  }
};