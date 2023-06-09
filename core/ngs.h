#include <range/v3/all.hpp>

#include "bfgs.h"

namespace Hexer {
emplate<typename Functor> class GaussSeidel {
  using Scalar = typename Functor::Scalar;

public:
  explicit GaussSeidel(Functor &functor, const std::vector<int> &label,
                       Eigen::Vector<Scalar, -1> &buffer,
                       BFGSOptions options = BFGSOptions())
      : _functor(functor) {
    _sync_x.resize(functor.inputs());

    int current_r = 0;
    _ranges.push_back(0);
    for (auto [i, r] : label | ranges::views::enumerate) {
      current_r += r;
      _ranges.push_back(current_r);
      auto buffer_range_x = Eigen::<Eigen::VectorXd>(
          _buffer_x.data() + _ranges[i] * 3, (_ranges[i + 1] - _ranges[i]) * 3);
      _solvers_list.push_back(BFGS<Functor>(functor, buffer_range_x, options));
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