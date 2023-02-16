#pragma once
#include <Expblas/ftensor.h>
#include <cinolib/meshes/meshes.h>

cinolib::DrawableTrimesh<> SphereGen(int n) {
  std::vector<Expblas::vec3d> verterx;
  std::vector<Expblas::vec3ui> faces;
  faces.reserve(n);

  verterx.push_back({0, 0, 1});  // 0
  verterx.push_back({0, 0, -1}); // 1
  verterx.push_back({1, 0, 0});  // 2
  verterx.push_back({-1, 0, 0}); // 3
  verterx.push_back({0, 1, 0});  // 4
  verterx.push_back({0, -1, 0}); // 5

  faces.push_back({0, 2, 4});
  faces.push_back({0, 4, 3});
  faces.push_back({0, 3, 5});
  faces.push_back({0, 5, 2});
  faces.push_back({1, 4, 2});
  faces.push_back({1, 3, 4});
  faces.push_back({1, 5, 3});
  faces.push_back({1, 2, 5});

  for (int i = 0; i < n; ++i) {
    int count = faces.size();
    for (int j = 0; j < count; ++j) {
      Expblas::vec3ui f = faces[j];
      faces.pop_back();
      for (int k = 0; k < 3; ++k) {
        verterx.push_back(Expblas::normalized(
            0.5 * (verterx[f[k]] + verterx[f[(k + 1) % 3]])));
        faces.push_back({verterx.size() - 1, f[k], f[(k + 1) % 3]});
      }
      uint s = verterx.size();
      faces.push_back({s - 3, s - 2, s - 1});
    }
  }

  cinolib::DrawableTrimesh<> mesh(
      std::vector<double>(verterx.data()->dataptr(),
                          verterx.data()->dataptr() + 3 * verterx.size()),
      std::vector<uint>(faces.data()->dataptr(),
                        faces.data()->dataptr() + faces.size() * 3));
  return mesh;
}