#pragma once

#include <cinolib/meshes/meshes.h>

#include <map>
#include <range/v3/all.hpp>

#include "base.h"

namespace Hexer {
enum class SortOrder { DescendOrder, AscendOrder };

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class Order
    : public CrtpExprBase<device, Order<device, ParamTuple>, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(const cinolib::AbstractMesh<M, V, E, P> &mesh, SortOrder order) {
    std::vector<int> ids = ranges::views::iota(0, int(mesh.num_verts())) |
                           ranges::to<std::vector<int>>;
    if (order == SortOrder::DescendOrder)
      return std::move(ids) |
             ranges::actions::sort([&](const auto &v1, const auto &v2) {
               return mesh.vert_valence(v1) > mesh.vert_valence(v2);
             });
    else
      return std::move(ids) |
             ranges::actions::sort([&](const auto &v1, const auto &v2) {
               return mesh.vert_valence(v1) < mesh.vert_valence(v2);
             });
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class VertexColoring
    : public CrtpExprBase<device, VertexColoring<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(const cinolib::AbstractMesh<M, V, E, P> &mesh,
            SortOrder orderRule) {
    auto order = Order()(mesh, orderRule).execute();

    std::vector<int> color(mesh.num_verts(), -1);
    int current_max_color = 0;
    int max_colors = 0;
    for (auto vid : ranges::views::iota(0, int(mesh.num_verts())))
      max_colors = std::max(max_colors, int(mesh.vert_valence(vid)));
    max_colors++;

    for (const auto vid : ranges::views::iota(0, int(mesh.num_verts()))) {
      // marks is used to store all the color which has already been
      // assigned to the surrounding vertex of vid
      std::vector<int> marks(max_colors, -1);
      int current_vid = order[vid];

      for (const auto &adjv : mesh.adj_v2v(current_vid))
        if (color[adjv] != -1)
          marks[color[adjv]] = 1;

      int c = 0;
      for (; c < max_colors; ++c)
        if (marks[c] == -1)
          break;
      color[current_vid] = c;
    }
    return color;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GraphColorMap
    : public CrtpExprBase<device, GraphColorMap<device, ParamTuple>,
                          ParamTuple> {
public:
  auto eval(const std::vector<int> &colors) {
    std::map<int, std::vector<int>> color_map;
    for (auto [vid, c] : colors | ranges::views::enumerate) {
      if (color_map.find(c) == color_map.end())
        color_map[c] = std::vector<int>();
      color_map[c].push_back(vid);
    }
    return color_map;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class RerangeVertexByColor
    : public CrtpExprBase<device, RerangeVertexByColor<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename MeshType>
  auto eval(const std::map<int, std::vector<int>> &color_map, MeshType &mesh) {
    std::vector<cinolib::vec3d> new_coords;
    std::vector<std::vector<uint>> new_polys;
    std::vector<int> old_new_vid_map(mesh.num_verts(), -1);
    new_polys.reserve(mesh.num_polys());
    new_coords.reserve(mesh.num_verts());

    int count_id = 0;
    for (const auto &[i, color_tuple] : color_map | ranges::views::enumerate) {
      for (auto &vid : color_tuple.second) {
        new_coords.push_back(mesh.vert(vid));
        old_new_vid_map[vid] = count_id;
        count_id++;
      }
    }
    for (uint pid = 0; pid < mesh.num_polys(); ++pid) {
      new_polys.push_back({});
      for (const auto &vid : mesh.poly_verts_id_const(pid))
        new_polys.back().push_back(old_new_vid_map[vid]);
    }

    std::vector<int> color_label = color_map |
                                   ranges::views::transform([](const auto &tp) {
                                     return tp.second.size();
                                   }) |
                                   ranges::to<std::vector<int>>();

    MeshType _mesh;
    _mesh.init(new_coords, new_polys, std::vector<int>{}, std::vector<int>{});
    return std::make_tuple(std::move(_mesh), std::move(color_label));
  }
};
} // namespace Hexer