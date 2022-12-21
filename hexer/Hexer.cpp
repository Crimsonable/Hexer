#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/drawable_polyhedralmesh.h>

#include "core/mesh.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh(
      "A:/MeshGeneration/Hexer/models/s01c_cube.vtk");
  auto smoother =
      Hexer::LaplacianMatrix<decltype(Hexer::uniform_weighter)>().operator()(
          &Hexer::uniform_weighter, mesh);
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}
