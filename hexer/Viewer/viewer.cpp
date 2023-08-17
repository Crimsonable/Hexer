#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh("../../../models/test_Step-1_PART-1-MESH-1-1f000.vtu");
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}