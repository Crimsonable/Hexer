#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

#include "core/deformation.h"
#include "core/transformation.h"

int main() {
  // cinolib::DrawablePolyhedralmesh<> mesh(
  //     "A:/MeshGeneration/Hexer/models/s01c_cube.vtk");
  cinolib::DrawablePolyhedralmesh<> mesh("D:/codes/Hexer/models/s01c_cube.vtk");
  mesh.update_bbox();
  cinolib::DrawableTrimesh<> surface_mesh;
  cinolib::DrawableTrimesh<> smooth_mesh;

  auto converter = Hexer::Convert2SurfaceMesh().operator()(surface_mesh);
  converter.execute(mesh);
  surface_mesh.init_drawable_stuff();
  surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  surface_mesh.update_bbox();
  surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  Hexer::LaplacianOptions options;
  options.n_smooth = 10;
  options.lambda = 0.01;
  options.method = Hexer::SmoothMethod::COTANGENT;

  smooth_mesh = surface_mesh;
  auto mat_gen = Hexer::LaplacianMatrix().operator()(options);
  auto sm = Hexer::LaplacianSmoother().operator()(mat_gen);
  sm.execute(smooth_mesh);
  smooth_mesh.init_drawable_stuff();
  smooth_mesh.translate(
      cinolib::vec3d(surface_mesh.bbox().delta_x() * 1.2, 0, 0));
  smooth_mesh.update_bbox();

  mesh.updateGL();
  surface_mesh.updateGL();
  smooth_mesh.updateGL();

  cinolib::GLcanvas gui;
  gui.push(&mesh);
  gui.push(&surface_mesh);
  gui.push(&smooth_mesh);
  return gui.launch();
}
