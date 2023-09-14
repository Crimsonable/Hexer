#include <cinolib/gl/glcanvas.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/tetgen_wrap.h>
#include <core/hexer_core.h>
#include <core/old_deformation.h>

#include <iostream>
#include <vector>

#include "hexer_core.h"

#include "test_utility.h"

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

void MeshAlign(cinolib::AbstractPolyhedralMesh<> &mesh) {
  auto surface_mesh = Hexer::Convert2SurfaceMesh()(mesh).execute();
  auto euler = Hexer::GlobalOrientationAlign()(surface_mesh).execute();
  Eigen::AngleAxisd rotation;
  rotation.fromRotationMatrix(Hexer::EulerToRotationMatrix(euler));
  cinolib::vec3d axis{rotation.axis()[0], rotation.axis()[1],
                      rotation.axis()[2]};
  mesh.rotate(axis, rotation.angle());
}

auto TetGenSphere() {
  cinolib::Polygonmesh<> sphere_surface =
      Hexer::LoopSubdivision()(unitOctahedron(), 3).execute();
  projectUnitSphere(sphere_surface);
  cinolib::DrawablePolygonmesh<> surface_mesh(sphere_surface.vector_verts(),
                                              sphere_surface.vector_polys());
  std::vector<cinolib::vec3d> v_out;
  std::vector<uint> e_in, p_out;
  cinolib::tetgen_wrap(surface_mesh.vector_verts(), surface_mesh.vector_polys(),
                       e_in, "", v_out, p_out);
  cinolib::DrawableTetmesh<> tet_mesh(v_out, p_out);

  return tet_mesh;
}

void TestEnergy() {
  // cinolib::Polyhedralmesh<> mesh("../../../models/s01c_cube.vtk");
  cinolib::Polyhedralmesh<> mesh(
      "../../../models/test_Step-1_PART-1-MESH-1-1f000.vtu");
  MeshAlign(mesh);

  Hexer::FacetNormalDeformOption options;
  Hexer::DeformEnergyOptions options2;
  options.sigma = 1;
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(
      mesh.vector_verts().data()->ptr(), mesh.num_verts() * 3);
  auto facet =
      Hexer::FacetNormalsEnergy()(mesh, options).execute(x, 0, false).first +
      Hexer::DeformEnergy()(mesh, options2).execute(x, 0, false).first;
  std::cout << facet << std::endl;
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
  cinolib::DrawablePolyhedralmesh<> mesh(
      "../../../models/test_Step-1_PART-1-MESH-1-1f000.vtu");

  // auto color_map =
  //     Hexer::VertexColoring()(mesh, Hexer::SortOrder::AscendOrder) |
  //     Hexer::GraphColorMap() | Hexer::evalOp();

  // int color_count = color_map.size();
  // ColorMap cmap;
  // auto colors = cmap.interp_n(color_count);

  // for (const auto &[c, idxs] : color_map) {
  //   for (auto id : idxs)
  //     mesh.vert_data(id).color =
  //         cinolib::Color(colors[c][0], colors[c][1], colors[c][0]);
  // }

  auto [_mesh, groups] =
      Hexer::VertexColoring()(mesh, Hexer::SortOrder::AscendOrder) |
      Hexer::GraphColorMap() | Hexer::RerangeVertexByColor()(mesh) |
      Hexer::evalOp();

  int color_count = groups.size();
  ColorMap cmap;
  auto colors = cmap.interp_n(color_count);

  int offset = 0;
  for (auto [i, gp] : groups | ranges::views::enumerate) {
    for (int vid = offset; vid < offset + gp; ++vid) {
      _mesh.vert_data(vid).color =
          cinolib::Color(colors[i][0], colors[i][1], colors[i][0]);
    }
    offset += gp;
  }

  // mesh.show_vert_color();
  _mesh.show_out_vert_color();
  _mesh.show_mesh_points();
  _mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&_mesh);
  return gui.launch();
}

int testMeshCopy() {
  cinolib::DrawablePolyhedralmesh<> mesh(
      "../../../models/test_Step-1_PART-1-MESH-1-1f000.vtu");

  cinolib::DrawablePolyhedralmesh<> _mesh;
  for (int i = 0; i < mesh.num_verts(); ++i)
    _mesh.vert_add(mesh.vert(i));
  for (int p = 0; p < mesh.num_polys(); ++p)
    _mesh.poly_add(mesh.adj_p2v(p));

  _mesh.update_f_normals();
  _mesh.update_v_normals();
  _mesh.show_out_vert_color();
  _mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&_mesh);
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
  Hexer::FacetNormalDeformOption options;

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

int testPolyCubeOptimization() {
  // auto mesh = TetGenSphere();
  //  cinolib::DrawablePolyhedralmesh<> mesh("../../../models/s01c_cube.vtk");
  cinolib::DrawablePolyhedralmesh<> mesh(
      "../../../models/test_Step-1_PART-1-MESH-1-1f000.vtu");
  hexer_timer([&]() { MeshAlign(mesh); }, "Mesh Align ");

  Hexer::DeformEnergyOptions d_options;
  d_options.s = 1e-5;
  Hexer::FacetNormalDeformOption f_options;

  hexer_timer(
      [&]() { Hexer::PolyCubeGen()(mesh, d_options, f_options, 1).execute(); },
      "PolyCube Gen ");

  mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}

void test_adj_v2p() {
  auto mesh = TetGenSphere();
  int count = 0;
  int count_min = 10000;
  for (int vid = 0; vid < mesh.num_verts(); ++vid) {
    if (mesh.adj_v2p(vid).size() > count)
      count = mesh.adj_v2p(vid).size();
    if (mesh.adj_v2p(vid).size() < count_min)
      count_min = mesh.adj_v2p(vid).size();
  }
  std::cout << count << std::endl;
  std::cout << count_min << std::endl;
  std::cout << mesh.vert(0) << std::endl;
}

auto debugDeformation() {
  auto mesh = TetGenSphere();
  for (int vid = 0; vid < mesh.num_verts(); ++vid) {
    int count = 0;
    for (int pid = 0; pid < mesh.num_polys(); ++pid) {
      if (mesh.poly_contains_vert(pid, vid))
        count++;
    }
    std::cout << count << std::endl;
  }
  mesh.show_out_wireframe(true);
  mesh.show_in_wireframe(true);
  mesh.save("rua.vtu");

  mesh.updateGL();
  cinolib::GLcanvas gui;
  gui.push(&mesh);
  return gui.launch();
}

int main() {
  // testPolyCubeOptimization();
  //    test_adj_v2p();
  //    TestLoopSubdivision();
  //    debugDeformation();
  //  debugDeformation();
  TestGraphColoring();
  // testMeshCopy();
  return 1;
}