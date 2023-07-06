#include <range/v3/all.hpp>

#include "bfgs.h"

namespace Hexer {
template <template <typename Mesh, typename DeformeE, typename FacetE>
          class Functor,
          typename... Args>
class GaussSeidel {
public:
  explicit GaussSeidel(const std::vector<int> &label,
                       Eigen::Vector<Scalar, -1> &buffer,
                       BFGSOptions options = BFGSOptions())
      : _ranges(label) {
    for (auto [i, r] : label | ranges::views::enumerate) {
      auto buffer_range_x = Eigen::Map<Eigen::VectorXd>(
          _buffer_x.data() + label[i] * 3, (_ranges[i + 1] - _ranges[i]) * 3);
    }
  }

  template <typename void init(Args &&...args) {
    for (auto [i, val] : _ranges | ranges::views::enumerate) {
      auto functor = Functor(std::forward<decltype(args)>(args)...);
      _solvers_list.push_back()
    }
  }

  int solve(const Eigen::VectorXd &x) {
    _sync_x = x;
    _buffer_x = x;

    for (auto &&[i, solver] : _solvers_list | ranges::views::enumerate) {
      auto range_x = Eigen::Map<Eigen::VectorXd>(
          _sync_x.data() + _ranges[i] * 3, (_ranges[i + 1] - _ranges[i]) * 3);
      solver.solve(range_x);
    }
  }

private:
  Functor &_functor;
  std::vector<BFGS<Functor>> _solvers_list;
  std::vector<int> _ranges;
  Eigen::VectorXd _sync_x;
  Eigen::VectorXd _buffer_x;
};
} // namespace Hexer