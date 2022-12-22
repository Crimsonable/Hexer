#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/drawable_polyhedralmesh.h>
#include <Eigen/Dense>

#include "core/deformation.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh(
      "D:/codes/Hexer/models/s01c_cube.vtk");
  auto smoother =
      Hexer::LaplacianMatrix().operator()(Hexer::UniformWeighter(), mesh);
  auto matrix = smoother.execute();
  Eigen::MatrixX<double> mat(2,3);
  auto mm=mat.outerStride();
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}
