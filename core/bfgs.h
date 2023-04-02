#pragma once
#include "base.h"
#include "solverbase.h"

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <omp.h>

namespace Hexer {
struct BFGSOptions {
  uint max_iteration = 1000;
  double rho = 0.55;
  double sigma = 0.4;
  double tol = 1e-6;
};

enum class DiffMode { Forward, Central };

template <DiffMode mode, typename Func>
int NumericalDiff(Func &&f, const Eigen::VectorXd &_x, Eigen::VectorXd &jac,
                  double epsfcn = 0.0) {
  using std::abs;
  using std::sqrt;
  /* Local variables */
  double h;
  int nfev = 0;
  int n = _x.size();
  const double eps =
      sqrt(((std::max)(epsfcn, Eigen::NumTraits<double>::epsilon())));
  Eigen::Vector<double, 1> val1, val2;
  Eigen::VectorXd x = _x;
  // TODO : we should do this only if the size is not already known
  val1.resize(f.values());
  val2.resize(f.values());

  // initialization
  if constexpr (mode == DiffMode::Forward) {
    // compute f(x)
    f(x, val1);
  }

// Function Body
//#pragma omp parallel for firstprivate(x, val1, val2, h)
  for (int j = 0; j < n; ++j) {
    h = eps * abs(x[j]);
    if (h == 0.) {
      h = eps;
    }
    if constexpr (mode == DiffMode::Forward) {
      x[j] += h;
      f(x, val2);
      x[j] = _x[j];
      jac.row(j) = (val2 - val1) / h;
    } else {
      x[j] += h;
      f(x, val2);
      x[j] -= 2 * h;
      f(x, val1);
      x[j] = _x[j];
      jac.row(j) = (val2 - val1) / (2 * h);
    }
  }
  return nfev;
}

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

    spdlog::drop("OptimalLog");
    logger = spdlog::basic_logger_mt("OptimalLog", "logs/opt.txt", true);
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
      Eigen::Vector<double, 1> new_f;
      Eigen::Vector<double, 1> old_f;
      _functor(x, old_f);
      for (; m < 20; ++m) {
        _functor(x + std::pow(_options.rho, m) * pk, new_f);
        if (new_f[0] <=
            old_f[0] + _options.sigma * std::pow(_options.rho, m) * gk.dot(pk))
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

      // spdlog::get("OptimalLog")
      //     ->info("iter: {} | TargetVal: {:03.2f}", k, old_f[0]);
      spdlog::info("iter: {} | TargetVal: {:03.2f}", k, old_f[0]);

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

  std::shared_ptr<spdlog::logger> logger;
};
} // namespace Hexer