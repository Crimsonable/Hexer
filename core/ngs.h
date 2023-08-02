#include <range/v3/all.hpp>
#include <range/v3/view/drop_last.hpp>

#include "bfgs.h"

namespace Hexer {
template <typename FunctorType, typename SolverType>
class GaussSeidel {
 public:
  template <typename... Args>
  explicit GaussSeidel(const std::vector<int> &label, Eigen::VectorXd &buffer,
                       BFGSOptions options, Args &&...args)
      : _ranges(label) {
    // to be cleared, 'label' is an array starts with 0 and ends with the total
    // number of vertices in the mesh
    _functors.reserve(label.size() - 1);
    _solvers_list.reserve(label.size() - 1);

    for (auto [i, r] :
         label | ranges::views::drop_last(1) | ranges::views::enumerate) {
      auto solver_buffer = (new Eigen::Map<Eigen::VectorXd>(
          buffer.data() + r * 3, (label[i + 1] - r) * 3));

      _functors.emplace_back(std::forward<decltype(args)>(args)..., buffer, r,
                             (label[i + 1] - r));
      _solvers_list.emplace_back(_functors.back(), *solver_buffer);
    }
  }

  template <typename VecX>
  int solve(VecX &x) {
    //_sync stores the real value that is being optimized
    _sync_x = x;
    _buffer_x = x;

    for (auto &&[i, solver] : _solvers_list | ranges::views::drop_last(1) |
                                  ranges::views::enumerate) {
      auto range_x = (new Eigen::Map<Eigen::VectorXd>(
          _sync_x.data() + _ranges[i] * 3, (_ranges[i + 1] - _ranges[i]) * 3));
      solver.solve(*range_x);
    }

    /*auto range_x = (new Eigen::Map<Eigen::VectorXd>(
        _sync_x.data() + 0, (_ranges[1] - _ranges[0]) * 3));
    _solvers_list[0].solve(*range_x);*/

    x = _sync_x;
    return 0;
  }

 private:
  std::vector<FunctorType> _functors;
  std::vector<SolverType> _solvers_list;
  std::vector<int> _ranges;
  Eigen::VectorXd _sync_x;
  Eigen::VectorXd _buffer_x;
};
}  // namespace Hexer