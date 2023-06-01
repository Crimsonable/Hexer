#include <range/v3/all.hpp>

#include "bfgs.h"

namespace Hexer {
emplate<typename Functor> class GaussSeidel {
  using Scalar = typename Functor::Scalar;

 public:
  explicit GaussSeidel(Functor& functor, BFGSOptions options = BFGSOptions())
      : _functor(functor) {
    _sync_x.resize(functor.inputs());
  }

  int solve(const Eigen::VectorXd& x) {
    _sync_x = x;
    for (auto&& [i, solver] : _solvers_list | ranges::views::enumerate) {
      auto range_x = Eigen::Map<Eigen::VectorXd>(_sync_x.data() +);
    }
  }

 private:
  Functor& _functor;
  std::vector<BFGS<Functor>> _solvers_list;
  std::vector<std::pair<int, int>> _ranges;
  Eigen::VectorXd _sync_x;
};
}  // namespace Hexer