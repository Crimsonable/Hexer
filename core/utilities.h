#pragma once

#include "base.h"

#include <cinolib/meshes/meshes.h>
#include <map>
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/filter.hpp>

namespace Hexer {

template <typename M, typename V, typename E, typename P>
auto DisconnectedVertex(const cinolib::AbstractMesh<M, V, E, P> &mesh,
                        std::vector<int> &current_set, int current_filter) {
  return current_set | ranges::views::filter([&](int vid) {
           return !mesh.vert_data(vid).flags[7] &&
                  ranges::find(mesh.adj_v2v(current_filter), vid) ==
                      ranges::end(mesh.adj_v2v(current_filter));
         });
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class ReIndex
    : public CrtpExprBase<device, ReIndex<device, ParamTuple>, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  auto eval(cinolib::AbstractMesh<M, V, E, P> &mesh) {
    std::map<int, std::vector<int>> color_map;
    auto reindex = ranges::views::iota(0, mesh.num_verts()) |
                   ranges::to<std::vector<int>>();
    reindex =
        std::move(reindex) | ranges::actions::sort([&](auto id1, auto id2) {
          return mesh.adj_v2v(id1).size() > mesh.adj_v2v(id2).size();
        });

    std::vector<int> filter_set = reindex;
    int current_filter = 0;
    for (auto color : ranges::iota(0)) {
      DisconnectedVertex(mesh, filter_set, current_filter);
    }
  }
};
} // namespace Hexer