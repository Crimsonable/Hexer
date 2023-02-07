#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

#include "core/deformation.h"
#include "core/transformation.h"

auto build_test_cube() {
  cinolib::DrawablePolygonmesh<> mesh;
  auto v1 = mesh.vert_add(cinolib::vec3d{0, 0, 0});
  auto v2 = mesh.vert_add(cinolib::vec3d{1, 0, 0});
  auto v3 = mesh.vert_add(cinolib::vec3d{1, 1, 0});
  auto v4 = mesh.vert_add(cinolib::vec3d{0, 1, 0});

  // auto e1 = mesh.edge_add(v1, v2);
  // auto e1 = mesh.edge_add(v2, v3);
  // auto e1 = mesh.edge_add(v3, v4);
  // auto e1 = mesh.edge_add(v4, v1);
  // auto e1 = mesh.edge_add(v1, v3);

  auto p1 = mesh.poly_add({v1, v3, v2});
  auto p2 = mesh.poly_add({v3, v4, v1});
  mesh.update_bbox();
  mesh.init_drawable_stuff();

  return mesh;
}

int main() {
  // cinolib::DrawablePolyhedralmesh<> mesh("../../../models/s01c_cube.vtk");
  // mesh.update_bbox();
  auto surface_mesh = build_test_cube();

  // cinolib::DrawableTrimesh<> surface_mesh;
  //  auto converter = Hexer::Convert2SurfaceMesh()(surface_mesh);
  //  hexer_timer([&]() { converter.execute(mesh); }, "Surface convert ");
  //  surface_mesh.init_drawable_stuff();
  //  surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  //  surface_mesh.update_bbox();
  //  surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  auto transformer = Hexer::GlobalOrientationAlign()(surface_mesh);
  hexer_timer([&]() { transformer.execute(); }, "Align operation ");

  Hexer::DeformationOptions options;
  options.sigma = 1;
  auto deformer = Hexer::GaussianSmoothFacetNormals()(options, surface_mesh);
  Eigen::Matrix3Xd gsn;
  hexer_timer([&]() { gsn = deformer.execute(); }, "Gaussian smoother ");
  auto gsn2 = Hexer::GaussianSmoothFacetNormals_naive(surface_mesh);
  surface_mesh.poly_data(0).color = cinolib::Color(1, 0, 0);

  std::cout << gsn << std::endl;
  std::cout << gsn2 << std::endl;

  surface_mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&surface_mesh);
  return gui.launch();
}
