#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

#include "core/deformation.h"
#include "core/transformation.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh("../../../models/s01c_cube.vtk");
  mesh.update_bbox();
  cinolib::DrawableTrimesh<> surface_mesh;

  auto converter = Hexer::Convert2SurfaceMesh()(surface_mesh);
  hexer_timer([&]() { converter.execute(mesh); }, "Surface convert ");
  surface_mesh.init_drawable_stuff();
  surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  surface_mesh.update_bbox();
  surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  Hexer::DeformationOptions options;
  options.sigma = 1;
  auto deformer = Hexer::GaussianSmoothFacetNormals()(options, surface_mesh);
  Eigen::Matrix3Xd gsn;
  hexer_timer([&]() { gsn = deformer.execute(); }, "Gaussian smoother ");
  auto gsn2 = Hexer::GaussianSmoothFacetNormals_naive(surface_mesh);

  std::cout << gsn.col(0) << std::endl;
  std::cout << gsn2.col(0) << std::endl;

  mesh.updateGL();
  surface_mesh.updateGL();

  cinolib::GLcanvas gui;
  gui.push(&surface_mesh);
  return gui.launch();
}
