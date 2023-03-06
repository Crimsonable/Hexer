#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

#include "test_utility.h"
#include <core/hexer_core.h>
#include <core/tessellation.h>

auto build_test_cube() {
  cinolib::DrawablePolygonmesh<> mesh;
  auto v1 = mesh.vert_add(cinolib::vec3d{0, 0, 0});
  auto v2 = mesh.vert_add(cinolib::vec3d{1, 0, 0});
  auto v3 = mesh.vert_add(cinolib::vec3d{1, 1, 0});
  auto v4 = mesh.vert_add(cinolib::vec3d{0, 1, 0});

  auto p1 = mesh.poly_add({v1, v3, v2});
  auto p2 = mesh.poly_add({v3, v4, v1});
  mesh.update_bbox();
  mesh.init_drawable_stuff();

  return mesh;
}

void applyColorByGauassianNormal(cinolib::DrawablePolygonmesh<> &mesh,
                                 const Eigen::Matrix3Xd &gsn) {
  for (int fid = 0; fid < mesh.num_polys(); ++fid) {
    cinolib::Color c(std::abs(gsn.col(fid).coeff(0)),
                     std::abs(gsn.col(fid).coeff(1)),
                     std::abs(gsn.col(fid).coeff(2)));
    mesh.poly_data(fid).color = c;
  }
}

int main0() {
  cinolib::Polygonmesh<> mesh =
      Hexer::LoopSubdivision()(unitOctahedron(), 5).execute();
  projectUnitSphere(mesh);

  cinolib::DrawablePolygonmesh<> dmesh(mesh.vector_verts(),
                                       mesh.vector_polys());

  dmesh.init_drawable_stuff();
  dmesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&dmesh);
  return gui.launch();
}

int main() {
  // cinolib::DrawablePolyhedralmesh<> mesh("../../../models/s01c_cube.vtk");
  // mesh.update_bbox();
  // auto surface_mesh = build_test_cube();
  cinolib::Polygonmesh<> raw_surface_mesh =
      Hexer::LoopSubdivision()(unitOctahedron(), 3).execute();
  projectUnitSphere(raw_surface_mesh);
  cinolib::DrawablePolygonmesh<> surface_mesh(raw_surface_mesh.vector_verts(),
                                              raw_surface_mesh.vector_polys());

  // cinolib::DrawableTrimesh<> surface_mesh;
  // auto converter = Hexer::Convert2SurfaceMesh()(surface_mesh);
  // hexer_timer([&]() { converter.execute(mesh); }, "Surface convert ");
  // surface_mesh.init_drawable_stuff();
  // surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  // surface_mesh.update_bbox();
  //  surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  auto transformer = Hexer::GlobalOrientationAlign()(surface_mesh);
  hexer_timer([&]() { transformer.execute(); }, "Align operation ");

  Hexer::DeformationOptions options;
  options.sigma = 1;
  auto deformer = Hexer::GaussianSmoothFacetNormals()(options, surface_mesh);
  Eigen::Matrix3Xd gsn;
  hexer_timer([&]() { gsn = deformer.execute(); }, "Gaussian smoother ");
  // auto gsn = Hexer::GaussianSmoothFacetNormals_naive(surface_mesh);
  // surface_mesh.poly_data(0).color = cinolib::Color(1, 0, 0);

  // std::cout << gsn << std::endl;
  applyColorByGauassianNormal(surface_mesh, gsn);

  surface_mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&surface_mesh);
  return gui.launch();
}
