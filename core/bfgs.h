#include "base.h"

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>

namespace Hexer {
struct BFGSOptions {
  uint max_iteration = 500;
  double rho = 0.55;
  double sigma = 0.4;
  double tol = 0.001;
};

template <typename Functor> class BFGS {
  using Scalar = typename Functor::Scalar;

public:
  BFGS(Functor &functor, BFGSOptions options = BFGSOptions())
      : _functor(functor), _options(options) {}

  int solve(Eigen::VectorX<Scalar> &x) {
    Bk = Eigen::MatrixX<Scalar>::Identity(x.rows(), x.rows());
    int k = 0;

    Eigen::LLT<Eigen::MatrixX<Scalar>> llt(x.rows(), x.rows());

    do {
      _functor.df(x, gk);
      llt = Eigen::LLT<Eigen::MatrixX<Scalar>>(Bk);
      pk = llt.solve(-gk);

      int m = 0;
      for (m < 20; ++m) {
        double new_f = _functor(x + std::pow(_options.rho, m) * pk);
        double old_f = _functor(x);
        if (new_f <=
            old_f + _options.sigma * std::pow(_options.rho, m) * gk.dot(pk))
          break;
      }

      _x = x + std::pow(_options.rho, m) * pk;
      _functor.df(_x, pk);
      gk = pk - gk;
      pk = _x - x;
      if (gk.dot(pk) > 0)
        Bk += (Scalar(1.0) / gk.dot(pk)) * gk * gk.transpose() -
              Bk * pk * pk.transpose() * Bk /
                  (pk.transpose().dot(pk.transpose() * Bk));
      x = _x;
    } while (gk.norm() > _options.tol && k < _options.max_iteration)
  }

private:
  Functor &_functor;
  BFGSOptions _options;
  Eigen::MatrixX<Scalar> Bk;
  Eigen::VectorX<Scalar> gk;
  Eigen::VectorX<Scalar> pk;
  Eigen::VectorX<Scalar> _x;
};
} // namespace Hexer