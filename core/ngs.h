#include <range/v3/all.hpp>

#include "bfgs.h"

namespace Hexer {
template <template <typename Mesh, typename DeformeE, typename FacetE>
          class Functor,
          typename... Args>
class GaussSeidel {
  // deduce the functor's type
  using FunctorType = typename Functor<Args...>;

public:
  explicit GaussSeidel(const std::vector<int> &label, Eigen::VectorXd &buffer,
                       Args &&...args, BFGSOptions options = BFGSOptions())
      : _ranges(label) {

    // to be cleared, 'label' is an array starts with 0 and ends with the total
    // number of vertices in the mesh
    for (auto [i, r] :
         label | ranges::views::drop_last | ranges::views::enumerate) {
      _functors.push_back(FunctorType(std::forward<decltype(args)>(args)...));

      auto solver_buffer = Eigen::Map<Eigen::VectorXd>(buffer.data() + r * 3,
                                                       (label[i + 1] - r) * 3);
      _solvers_list.push_back(
          BFGS<FunctorType>(_functors.back(), solver_buffer));
    }
  }

  int solve(Eigen::VectorXd &x) {
    //_sync stores the real value that is being optimized
    _sync_x = x;
    _buffer_x = x;

    for (auto &&[i, solver] : _solvers_list | ranges::views::enumerate) {
      auto range_x = Eigen::Map<Eigen::VectorXd>(
          _sync_x.data() + _ranges[i] * 3, (_ranges[i + 1] - _ranges[i]) * 3);
      solver.solve(range_x);
    }

    x = _sync_x;
  }

private:
  std::vector<FunctorType> _functors;
  std::vector<BFGS<FunctorType>> _solvers_list;
  std::vector<int> _ranges;
  Eigen::VectorXd _sync_x;
  Eigen::VectorXd _buffer_x;
};
} // namespace Hexer