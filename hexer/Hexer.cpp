#include <iostream>
#include <vector>

#include <cinolib/meshes/drawable_polyhedralmesh.h>
#include <cinolib/gl/glcanvas.h>

#include "core/mesh.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh("A:/MeshGeneration/Hexer/models/s01c_cube.vtk");
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}
