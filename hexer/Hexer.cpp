#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/drawable_polyhedralmesh.h>

#include "core/deformation.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh("D:/codes/Hexer/models/s01c_cube.vtk");
  cinolib::DrawablePolygonmesh<> surface_mesh;

  std::cout << "surface faces count: " << mesh.get_surface_faces().size()
            << std::endl;

  auto smoother =
      Hexer::LaplacianMatrix().operator()(Hexer::UniformWeighter(), mesh);
  auto matrix = smoother.execute();

  auto converter = Hexer::Convert2SurfaceMesh().operator()(surface_mesh);
  converter.execute(mesh);
  surface_mesh.init_drawable_stuff();
  std::cout << "convert surface mesh faces count: " << surface_mesh.num_polys()
            << std::endl;
  surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));

  cinolib::GLcanvas gui;
  //gui.push(&mesh);
  gui.push(&surface_mesh);
  return gui.launch();
}
