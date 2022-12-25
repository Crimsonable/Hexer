#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/drawable_polyhedralmesh.h>

#include "core/deformation.h"

int main() {
  cinolib::DrawablePolyhedralmesh<> mesh(
      "A:/MeshGeneration/Hexer/models/s01c_cube.vtk");
  mesh.update_bbox();
  //cinolib::DrawablePolygonmesh<> surface_mesh;
  cinolib::Polygonmesh<> surface_mesh;
  //cinolib::DrawablePolygonmesh<> smooth_mesh;
  cinolib::Polygonmesh<> smooth_mesh;

  std::cout << "surface faces count: " << mesh.get_surface_faces().size()
            << std::endl;

  auto converter = Hexer::Convert2SurfaceMesh().operator()(surface_mesh);
  converter.execute(mesh);

  Hexer::LaplacianOptions options;
  // Hexer::LaplacianSmoother().eval(smooth_mesh, options,
  //                                 Eigen::SparseMatrix<double>(),
  //                                 surface_mesh);

  auto tp = std::make_tuple(smooth_mesh, options,
                            Eigen::SparseMatrix<double>(), surface_mesh);
  // auto sm = Hexer::LaplacianSmoother();
  // sm.execute_helper(std::make_index_sequence<4>{}, tp);

  auto mat_gen = Hexer::LaplacianMatrix().operator()(options);
  auto sm =
      Hexer::LaplacianSmoother().operator()(smooth_mesh, mat_gen);
  sm.execute(surface_mesh);

  // surface_mesh.init_drawable_stuff();
  // std::cout << "convert surface mesh faces count: " << surface_mesh.num_polys()
  //           << std::endl;
  // surface_mesh.translate(cinolib::vec3d(mesh.bbox().delta_x() * 1.2, 0, 0));
  // surface_mesh.update_bbox();
  // surface_mesh.poly_set_color(cinolib::Color(0.3098, 0.7647, 0.9686));

  // mesh.updateGL();
  // surface_mesh.updateGL();

  // cinolib::GLcanvas gui;
  // gui.push(&mesh);
  // gui.push(&surface_mesh);
  return gui.launch();
}
