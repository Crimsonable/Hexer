#pragma once
#include "base.h"
#include "solverbase.h"

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

namespace Hexer {
struct BFGSOptions {
  uint max_iteration = 1000;
  double rho = 0.55;
  double sigma = 0.4;
  double tol = 1e-6;
};

template <typename Functor> class BFGS {
  using Scalar = typename Functor::Scalar;

public:
  BFGS(Functor &functor, BFGSOptions options = BFGSOptions())
      : _functor(functor), _options(options) {
    uint m = functor.inputs();
    Bk.resize(m, m);
    gk.resize(m);
    pk.resize(m);
    _x.resize(m);
  }

  int solve(Eigen::VectorX<Scalar> &x) {
    Bk = Eigen::MatrixX<Scalar>::Identity(x.rows(), x.rows());
    int k = 0;

    Eigen::LLT<Eigen::MatrixX<Scalar>> llt(x.rows());

    do {
      _functor.df(x, gk);
      llt.compute(Bk);
      pk = llt.solve(-gk);

      int m = 0;
      Scalar new_f;
      Scalar old_f;
      _functor(x, old_f);
      for (; m < 20; ++m) {
        _functor(x + std::pow(_options.rho, m) * pk, new_f);
        if (new_f <=
            old_f + _options.sigma * std::pow(_options.rho, m) * gk.dot(pk))
          break;
      }

      _x = x + std::pow(_options.rho, m) * pk;
      _functor.df(_x, pk);
      gk = pk - gk;
      pk = _x - x;
      if (gk.dot(pk) > 0) {
        Scalar coeff1 = Scalar(1.0) / gk.dot(pk);
        Scalar coeff2 = Scalar(1.0) / pk.transpose().dot(pk.transpose() * Bk);

        Bk += coeff1 * gk * gk.transpose() -
              coeff2 * Bk * pk * pk.transpose() * Bk;
      }
#ifdef _DEBUG
      spdlog::get("OptimalLog")
          ->info("iter: {} | TargetVal: {:03.2f} | x: "
                 "({:03.2f},{:03.2f},{:03.2f})",
                 k, old_f, x[0], x[1], x[2]);
#endif

      x = _x;
      k++;
    } while (gk.norm() > _options.tol && k < _options.max_iteration);

    return 0;
  }

private:
  Functor &_functor;
  BFGSOptions _options;
  Eigen::MatrixX<Scalar> Bk;
  Eigen::VectorX<Scalar> gk;
  Eigen::VectorX<Scalar> pk;
  Eigen::VectorX<Scalar> _x;

#ifdef _DEBUG
  std::shared_ptr<spdlog::logger> logger =
      spdlog::basic_logger_mt("OptimalLog", "logs/opt.txt", true);
#endif
};
} // namespace Hexer