#include "expr.h"

#include <Expblas/graph_funcs.h>
#include <cinolib/meshes/meshes.h>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>

namespace Hexer {
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {
  typedef _Scalar Scalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };
  typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  const int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }
};

template <typename M = cinolib::Mesh_std_attributes,
          typename V = cinolib::Vert_std_attributes,
          typename E = cinolib::Edge_std_attributes,
          typename P = cinolib::Polygon_std_attributes>
struct GlobalOrientationAlignFunctor : public Functor<double> {
  GlobalOrientationAlignFunctor(cinolib::AbstractMesh<M, V, E, P> &mesh)
      : _mesh(mesh), Functor<double>(3, 1) {}

  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) {
    Eigen::AngleAxisd yaw_angle(x[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_angle(x[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd roll_angle(x[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation =
        (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();

    fvec[0] = 0.0;
    auto normals = _mesh.vector_poly_normals();
    Eigen::Matrix3Xd n =
        Eigen::Map<Eigen::Matrix3Xd>(normals.data(), _mesh.num_polys());
    Eigen::Matrix3Xd Rn = rotation * n;
    Rn = Rn.pow(2);

  }

  Eigen::Matrix3d _trans{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};
  cinolib::AbstractMesh<M, V, E, P> &_mesh;
};

class GlobalOrientationAlign
    : public CrtpExprBase<Device::CPU, GlobalOrientationAlign> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(cinolib::AbstractMesh<M, V, E, P> &mesh) {
    double row(0), pitch(0), yaw(0);

    Eigen::Matrix3d rotation = yaw_angle * pitch_angle * row_angle;
  }
};

} // namespace Hexer