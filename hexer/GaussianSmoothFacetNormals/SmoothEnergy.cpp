#include <iostream>
#include <vector>

#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>

#include "test_utility.h"
#include <core/hexer_core.h>

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

void TestEnergy() {
  cinolib::Polyhedralmesh<> mesh("../../../models/s01c_cube.vtk");

  Hexer::DeformationOptions options;
  options.sigma = 1;
  auto facet = Hexer::FacetNormalsEnergy()(mesh, options).execute();
}

int TestLoopSubdivision() {
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

int TestGraphColoring() {
  cinolib::Polygonmesh<> raw_surface_mesh =
      Hexer::LoopSubdivision()(unitOctahedron(), 3).execute();
  projectUnitSphere(raw_surface_mesh);
  cinolib::DrawablePolygonmesh<> surface_mesh(raw_surface_mesh.vector_verts(),
                                              raw_surface_mesh.vector_polys());

  auto color =
      Hexer::VertexColoring()(surface_mesh, Hexer::SortOrder::DescendOrder)
          .execute();
  bool check_flag = true;
  for (auto vid : ranges::views::iota(0, int(surface_mesh.num_verts()))) {
    for (auto adj_vid : surface_mesh.adj_v2v(vid))
      if (color[adj_vid] == color[vid]) {
        check_flag = false;
        break;
      }
    if (!check_flag)
      break;
  }
  std::cout << "Graph Coloring Check: " << check_flag << std::endl;

  int color_count = ranges::max(color);
  cinolib::vec3d color_ori{1, 1, 0};
  cinolib::vec3d color_ed{0, 0, 1};
  auto color_gap = 1.0 / color_count * (color_ed - color_ori);
  for (auto vid : ranges::views::iota(0, int(surface_mesh.num_verts()))) {
    auto current_color = double(color[vid]) * color_gap + color_ori;
    surface_mesh.vert_data(vid).color =
        cinolib::Color(current_color[0], current_color[1], current_color[2]);
  }

  surface_mesh.show_vert_color();
  surface_mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&surface_mesh);
  return gui.launch();
}

int testGSN() {
  cinolib::Polygonmesh<> raw_surface_mesh =
      Hexer::LoopSubdivision()(unitOctahedron(), 3).execute();
  projectUnitSphere(raw_surface_mesh);
  cinolib::DrawablePolygonmesh<> surface_mesh(raw_surface_mesh.vector_verts(),
                                              raw_surface_mesh.vector_polys());

  auto transformer = Hexer::GlobalOrientationAlign()(surface_mesh);
  hexer_timer([&]() { transformer.execute(); }, "Align operation ");

  Hexer::DeformationOptions options;
  options.sigma = 1;
  auto deformer = Hexer::GaussianSmoothFacetNormals()(surface_mesh, options);
  Eigen::Matrix3Xd gsn;
  hexer_timer([&]() { gsn = deformer.execute(); }, "Gaussian smoother ");
  applyColorByGauassianNormal(surface_mesh, gsn);

  surface_mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&surface_mesh);
  return gui.launch();
}

int main() {
  TestGraphColoring();
  return 1;
}