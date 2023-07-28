#include "bfgs.h"
#include "expr.h"
#include "threadpool.h"

// #include <Expblas/graph_funcs.h>
#include <cinolib/meshes/meshes.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>

namespace Hexer {
inline Eigen::Matrix3d EulerToRotationMatrix(const Eigen::VectorXd &euler) {
  Eigen::AngleAxisd yaw_angle(euler[2], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll_angle(euler[0], Eigen::Vector3d::UnitX());
  return (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
}

// derivative of Euler rotation matrix
inline Eigen::Matrix3d dfRotationTheta_x(double x, double y, double z) {
  double sx = std::sin(x);
  double cx = std::cos(x);
  double sy = std::sin(y);
  double cy = std::cos(y);
  double sz = std::sin(z);
  double cz = std::cos(z);

  return Eigen::Matrix3d{{0, cx * sy * cz + sx * sz, cx * sz - sx * sy * cz},
                         {0, cx * sy * sz - sx * cz, -sx * sy * sz - cx * cz},
                         {0, cx * cy, -sx * cy}};
}

inline Eigen::Matrix3d dfRotationTheta_y(double x, double y, double z) {
  double sx = std::sin(x);
  double cx = std::cos(x);
  double sy = std::sin(y);
  double cy = std::cos(y);
  double sz = std::sin(z);
  double cz = std::cos(z);

  return Eigen::Matrix3d{{-sy * cz, sx * cy * cz, cx * cy * cz},
                         {-sy * sz, sx * cy * sz, cx * cy * sz},
                         {-cy, -sx * sy, -cx * sy}};
}

inline Eigen::Matrix3d dfRotationTheta_z(double x, double y, double z) {
  double sx = std::sin(x);
  double cx = std::cos(x);
  double sy = std::sin(y);
  double cy = std::cos(y);
  double sz = std::sin(z);
  double cz = std::cos(z);

  return Eigen::Matrix3d{
      {-cy * sz, -sx * sy * sz - cx * cz, cz * sx - cx * sy * sz},
      {cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz},
      {0, 0, 0}};
}

template <typename M = cinolib::Mesh_std_attributes,
          typename V = cinolib::Vert_std_attributes,
          typename E = cinolib::Edge_std_attributes,
          typename P = cinolib::Polygon_std_attributes>
struct GlobalOrientationAlignFunctor : public Functor<double> {
  GlobalOrientationAlignFunctor(cinolib::AbstractMesh<M, V, E, P> &mesh)
      : _mesh(mesh), Functor<double>(3, 1), _pool(3) {
    _normals = _mesh.vector_poly_normals();
    _Rn = Eigen::Matrix3Xd(3, _mesh.num_polys());
    _ERn = Eigen::Matrix3Xd(3, _mesh.num_polys());

    _pool.init();
  }

  ~GlobalOrientationAlignFunctor() { _pool.shutdown(); }

  // target function: f(n)=nx*ny+nx*nz+ny*nz
  int operator()(const Eigen::VectorXd &x, Eigen::Vector<double, 1> &fvec,
                 Eigen::VectorXd &fjac, bool gradient) {
    Eigen::Matrix3d rotation = EulerToRotationMatrix(x);
    Eigen::Matrix3Xd Rn = Eigen::Map<Eigen::Matrix3Xd>(_normals.data()->ptr(),
                                                       3, _normals.size());

    _Rn = rotation * Rn;
    _Rn = _Rn.cwiseProduct(_Rn);
    _ERn = _trans * _Rn;
    fvec[0] = _Rn.cwiseProduct(_ERn).sum();

    if (gradient)
      df(x, fjac);

    return 0;
  }

  int df(const Eigen::VectorXd &x, Eigen::VectorXd &fjac) {
    Eigen::Matrix3d rotation = EulerToRotationMatrix(x);
    Eigen::Matrix3Xd n = Eigen::Map<Eigen::Matrix3Xd>(_normals.data()->ptr(), 3,
                                                      _normals.size());

    _Rn = rotation * n;
    _ERn = (_trans + _trans.transpose()) * _Rn.cwiseProduct(_Rn);
    _ERn = _ERn.cwiseProduct(_Rn);
    Eigen::VectorXd coeffs =
        Eigen::Map<Eigen::VectorXd>(_Rn.data(), _normals.size());
    coeffs = _ERn.colwise().sum().transpose();

    Eigen::VectorXd coeffs2 =
        Eigen::Map<Eigen::VectorXd>(_ERn.data(), _normals.size());

    auto future1 = _pool.submit([&]() {
      fjac[0] = coeffs
                    .cwiseProduct((dfRotationTheta_x(x[0], x[1], x[2]) * n)
                                      .colwise()
                                      .sum()
                                      .transpose())
                    .sum();
    });

    auto future2 = _pool.submit([&]() {
      fjac[1] = coeffs
                    .cwiseProduct((dfRotationTheta_y(x[0], x[1], x[2]) * n)
                                      .colwise()
                                      .sum()
                                      .transpose())
                    .sum();
    });

    auto future3 = _pool.submit([&]() {
      fjac[2] = coeffs
                    .cwiseProduct((dfRotationTheta_z(x[0], x[1], x[2]) * n)
                                      .colwise()
                                      .sum()
                                      .transpose())
                    .sum();
    });

    future1.get();
    future2.get();
    future3.get();

    return 0;
  }

  Eigen::Matrix3d _trans{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
  Eigen::Matrix3Xd _Rn;
  Eigen::Matrix3Xd _ERn;
  cinolib::AbstractMesh<M, V, E, P> &_mesh;
  std::vector<cinolib::vec3d> _normals;

  ThreadPool _pool;
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GlobalOrientationAlign
    : public CrtpExprBase<device, GlobalOrientationAlign<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(cinolib::AbstractMesh<M, V, E, P> &mesh) {
    Eigen::VectorXd euler{{0, 0, 0}};
    Eigen::VectorXd buffer_euler = euler;
    GlobalOrientationAlignFunctor functor(mesh);
    BFGS<decltype(functor), decltype(buffer_euler)> solver(functor, buffer_euler);
    auto info = solver.solve(euler);

    return euler;
    Eigen::AngleAxisd rotation;
    rotation.fromRotationMatrix(EulerToRotationMatrix(euler));
    cinolib::vec3d axis{rotation.axis()[0], rotation.axis()[1],
                        rotation.axis()[2]};
    mesh.rotate(axis, rotation.angle());
  }
};

} // namespace Hexer