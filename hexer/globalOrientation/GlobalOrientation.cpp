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
  cinolib::DrawableTrimesh<> ori_mesh;

  auto converter = Hexer::Convert2SurfaceMesh().operator()(surface_mesh);
  converter.execute(mesh);
  surface_mesh.init_drawable_stuff();
  surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  surface_mesh.update_bbox();
  surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  ori_mesh = surface_mesh;
  auto orientation_opt = Hexer::GlobalOrientationAlign();
  orientation_opt.execute(ori_mesh);
  ori_mesh.update_bbox();

  mesh.updateGL();
  surface_mesh.updateGL();
  ori_mesh.updateGL();

  cinolib::GLcanvas gui;
  gui.push(&mesh);
  gui.push(&surface_mesh);
  gui.push(&ori_mesh);
  return gui.launch();
}