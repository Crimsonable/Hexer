#pragma once
#include "bfgs.h"
#include "expr.h"
#include "mesh.h"

#include <cinolib/meshes/meshes.h>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <numeric>
#include <range/v3/all.hpp>

namespace Hexer {
enum class SmoothMethod { UNIFORM, COTANGENT };

struct LaplacianOptions {
  int n_smooth = 1;
  double lambda = 0.001;
  SmoothMethod method = SmoothMethod::UNIFORM;
};

class UniformWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<Eigen::Triplet<double>> &entries, int nv,
                  int current_vid,
                  const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    auto ring = mesh.vert_n_ring(current_vid, 1);
    std::vector<int> offset;

    for (int i = 0; i < nv; ++i)
      offset.push_back(i * mesh.num_verts());

    for (auto &&_offset : offset)
      entries.emplace_back(current_vid + _offset, current_vid + _offset,
                           ring.size());

    for (auto &&iter : ring)
      for (auto &&_offset : offset)
        entries.emplace_back(current_vid + _offset, iter + _offset, -1.0);
  }
};

class CotangentWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<Eigen::Triplet<double>> &entries, int nv,
                  int current_vid, const cinolib::Trimesh<M, V, E, P> &mesh) {
    std::vector<int> offset;
    for (int i = 0; i < nv; ++i)
      offset.push_back(i * mesh.num_verts());

    // caculate cotangent weight using cinolib internal function
    std::vector<std::pair<int, double>> weight;
    mesh.vert_weights_cotangent(current_vid, weight);

    double sum =
        std::accumulate(weight.begin(), weight.end(), 0.0,
                        [](double w1, const std::pair<int, double> &w) {
                          return w1 + w.second;
                        });

    for (auto &&_offset : offset)
      entries.emplace_back(current_vid + _offset, current_vid + _offset, sum);

    for (auto &&w : weight)
      for (auto &&_offset : offset)
        entries.emplace_back(current_vid + _offset, w.first + _offset,
                             -w.second);
  }
};

class AverageWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<Eigen::Triplet<double>> &entries, int nv,
                  int current_vid, const cinolib::Trimesh<M, V, E, P> &mesh) {}
};

// caculate Laplacian matrix using given method(cotangent,uniform)
template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LaplacianMatrix
    : public CrtpExprBase<Device::CPU, LaplacianMatrix<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const LaplacianOptions &options,
                   cinolib::AbstractMesh<M, V, E, P> &mesh) {
    int n = mesh.num_verts();
    auto Laplacian = Eigen::SparseMatrix<double>(n * 3, n * 3);
    std::vector<Eigen::Triplet<double>> entries;

    if (options.method == SmoothMethod::UNIFORM) {
      auto weighter = UniformWeighter();
      for (int vid = 0; vid < mesh.num_verts(); ++vid) {
        weighter(entries, 3, vid, mesh);
      }
    } else if (options.method == SmoothMethod::COTANGENT) {
      // cotangent only support triangle mesh, test mesh type using dynamic_cast
      const cinolib::Trimesh<> *tri_mesh =
          dynamic_cast<const cinolib::Trimesh<M, V, E, P> *>(&mesh);
      if (tri_mesh == nullptr) {
        assert(false &&
               "mesh is not trimesh, doesn't support cotangent weight.");
      }
      auto weighter = CotangentWeighter();
      for (int vid = 0; vid < tri_mesh->num_verts(); ++vid) {
        weighter(entries, 3, vid, *tri_mesh);
      }
    }
    Laplacian.setFromTriplets(entries.begin(), entries.end());
    return std::make_tuple(std::move(options), std::move(Laplacian),
                           std::ref(mesh));
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LaplacianSmoother
    : public CrtpExprBase<device, LaplacianSmoother<device, ParamTuple>,
                          ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const LaplacianOptions &options,
                   Eigen::SparseMatrix<double> &&Laplacian,
                   cinolib::AbstractMesh<M, V, E, P> &mesh) {
    std::vector<double> vertices_coords;
    int n = mesh.num_verts();
    vertices_coords.resize(n * 3);

    for (int vid = 0; vid < n; ++vid) {
      auto p = mesh.vert(vid);
      vertices_coords[vid] = p.x();
      vertices_coords[vid + n] = p.y();
      vertices_coords[vid + 2 * n] = p.z();
    }
    Eigen::VectorXd v =
        Eigen::Map<Eigen::VectorXd>(vertices_coords.data(), n * 3);

    for (int i = 0; i < options.n_smooth; ++i) {
      v -= options.lambda * Laplacian * v;
      if (options.method == SmoothMethod::COTANGENT)
        Laplacian =
            std::move(std::get<1>(LaplacianMatrix<>::eval(options, mesh)));
    }

    for (int vid = 0; vid < mesh.num_verts(); ++vid) {
      mesh.vert(vid) =
          cinolib::vec3d(v.coeff(vid), v.coeff(vid + n), v.coeff(vid + 2 * n));
    }
  }
};

struct FacetNormalDeformOption {
  double sigma = 0.01;
};

template <typename M, typename V, typename E, typename P>
inline auto poly_centroid(cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                          int fid) {
  auto tmp = mesh.poly_centroid(fid);
  Eigen::Vector3d p;

  p(0) = tmp[0];
  p(1) = tmp[1];
  p(2) = tmp[2];

  return p;
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GaussianDistanceWeight
    : public CrtpExprBase<device, GaussianDistanceWeight<device, ParamTuple>,
                          ParamTuple> {
public:
  auto eval(const Eigen::Matrix3Xd &normals, const Eigen::Matrix3Xd &centers,
            const Eigen::VectorXd &areas, double sigma, int fid) {
    Eigen::Vector3d gn{0, 0, 0};

    auto poly_center_minus =
        -0.5 / std::pow(sigma, 2) *
        (centers.colwise() - centers.col(fid)).colwise().norm();

    auto exp_distance =
        areas.array().transpose() * Eigen::exp(poly_center_minus.array());

    gn += (normals.array().rowwise() * exp_distance)
              .rowwise()
              .sum()
              .matrix()
              .transpose();
    return gn;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GaussianSmoothFacetNormals
    : public CrtpExprBase<
          device, GaussianSmoothFacetNormals<device, ParamTuple>, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                   const FacetNormalDeformOption &options) {
    std::vector<cinolib::vec3d> ns = mesh.vector_poly_normals();

    Eigen::Matrix3Xd normals =
        Eigen::Map<Eigen::Matrix3Xd>(ns.data()->ptr(), 3, ns.size());

    Eigen::Matrix3Xd poly_centers;
    poly_centers.resize(3, mesh.num_polys());

    Eigen::VectorXd area;
    area.resize(mesh.num_polys());
    for (int fid = 0; fid < mesh.num_polys(); ++fid) {
      area.coeffRef(fid) = mesh.poly_area(fid);
      poly_centers.col(fid) = poly_centroid(mesh, fid);
    }

    Eigen::Matrix3Xd gsn = Eigen::Matrix3Xd::Zero(3, mesh.num_polys());

    auto gaussian_weighter =
        GaussianDistanceWeight()(normals, poly_centers, area, options.sigma);

    Eigen::Index _idx;
    for (int fid = 0; fid < mesh.num_polys(); ++fid) {
      Eigen::Vector3d _v = gaussian_weighter.execute(fid);
      _v.cwiseAbs().maxCoeff(&_idx);
      gsn.col(fid).coeffRef(_idx) = std::signbit(_v.coeff(_idx)) ? -1.0f : 1.0f;
    }

    return gsn;
  }
};

template <typename M, typename V, typename E, typename P>
auto GaussianSmoothFacetNormals_naive(
    cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
  Eigen::Matrix3Xd gsn;
  gsn.resize(3, mesh.num_polys());
  auto ns = mesh.vector_poly_normals();

  for (int fid = 0; fid < mesh.num_polys(); ++fid) {
    cinolib::vec3d n{0, 0, 0};
    Eigen::Vector3d p = poly_centroid(mesh, fid);
    for (int fid2 = 0; fid2 < mesh.num_polys(); ++fid2) {
      Eigen::Vector3d p2 = poly_centroid(mesh, fid);
      n = n +
          mesh.poly_area(fid2) * std::exp(-0.5 * (p2 - p).norm()) * ns[fid2];
    }
    gsn.col(fid).coeffRef(0) = n[0];
    gsn.col(fid).coeffRef(1) = n[1];
    gsn.col(fid).coeffRef(2) = n[2];
  }
  return gsn;
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class FacetNormalsEnergy
    : public CrtpExprBase<device, FacetNormalsEnergy<device, ParamTuple>,
                          ParamTuple> {
  Eigen::Matrix3Xd _gsn;
  std::map<int, int> _face_map;

public:
  template <typename... Args>
  FacetNormalsEnergy(Args &&...args)
      : CrtpExprBase<device, FacetNormalsEnergy<device, ParamTuple>,
                     ParamTuple>(std::forward<decltype(args)>(args)...) {}

  template <typename M, typename V, typename E, typename F, typename P>
  HEXER_INLINE auto
  eval(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
       FacetNormalDeformOption options, const Eigen::VectorXd &x, int pid) {
    double n_gsn = 0.0;
    for (auto f : mesh.poly_faces_id(pid))
      if (mesh.face_data(f).flags[0]) {
        auto &vid = mesh.adj_f2v(f);
        auto v0 = x.block<3, 1>(vid[0] * 3, 0);
        auto v1 = x.block<3, 1>(vid[1] * 3, 0);
        auto v2 = x.block<3, 1>(vid[2] * 3, 0);

        n_gsn +=
            ((v1 - v0).cross(v2 - v0).normalized() - _gsn.col(_face_map[f]))
                .squaredNorm();
      }
    return n_gsn;
  }

  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
            FacetNormalDeformOption options, const Eigen::VectorXd &x) {
    if (_gsn.size() == 0) {
      auto surface_mesh = Convert2SurfaceMesh()(mesh).execute();
      _gsn = GaussianSmoothFacetNormals()(surface_mesh, options).execute();
      auto index_raw = mesh.get_surface_faces();

      for (const auto &[i, f] : index_raw | ranges::views::enumerate)
        _face_map[f] = i;
    }

    double n_gsn = 0;
    for (int pid = 0; pid < mesh.num_polys(); ++pid)
      n_gsn += this->eval(mesh, options, x, pid);

    return n_gsn;
  }
};

struct DeformEnergyOptions {
  double alpha = 0.5;
  double s = 1.0;
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class DeformEnergy
    : public CrtpExprBase<device, DeformEnergy<device, ParamTuple>,
                          ParamTuple> {

  Eigen::Matrix3Xd A_0;

public:
  template <typename... Args>
  DeformEnergy(Args &&...args)
      : CrtpExprBase<device, DeformEnergy<device, ParamTuple>, ParamTuple>(
            std::forward<decltype(args)>(args)...) {}

  // poly's deformation affine matrix is [vp-vq | vp-vr | vp-vs]
  template <typename Mat>
  HEXER_INLINE auto poly_affine(const Eigen::VectorXd &x, int pid,
                                const std::vector<uint> &adj_p2v,
                                Eigen::MatrixBase<Mat> &mat) {
    for (int i = 0; i < adj_p2v.size() - 1; ++i)
      if constexpr (mat.RowsAtCompileTime == 3 && mat.ColsAtCompileTime == 3)
        mat.col(i) = x.block<3, 1>(3 * adj_p2v[0], 0) -
                     x.block<3, 1>(3 * adj_p2v[i + 1], 0);
      else
        mat.block<3, 3>(0, 3 * pid).col(i) =
            x.block<3, 1>(3 * adj_p2v[0], 0) -
            x.block<3, 1>(3 * adj_p2v[i + 1], 0);
  }
  template <typename M, typename V, typename E, typename F, typename P>
  HEXER_INLINE auto
  eval(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
       DeformEnergyOptions options, const Eigen::VectorXd &x, int pid) {
    Eigen::Matrix3d A_1;
    poly_affine(x, pid, mesh.adj_p2v(pid), A_1);
    auto A_expr = A_1 * A_0.block<3, 3>(0, pid * 3);
    auto A_inv = A_expr.inverse();
    double conformal = 0.125 * (A_expr.squaredNorm() * A_inv.squaredNorm() - 1);
    double A_det = A_expr.determinant();
    double volumetric = 0.5 * (A_det + 1.0 / A_det);
    return std::exp(options.s *
                    (options.alpha * conformal + options.alpha * volumetric));
  }

  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
            DeformEnergyOptions options, const Eigen::VectorXd &x) {

    // A_0 is constant during the calculation, if A_0's size equals to 0, then
    // calculate A_0 once.
    if (A_0.size() == 0) {
      A_0.resize(3, mesh.num_polys() * 3);

      for (int pid = 0; pid < mesh.num_polys(); ++pid) {
        poly_affine(x, pid, mesh.adj_p2v(pid), A_0);
        A_0.block<3, 3>(0, 3 * pid) =
            A_0.block<3, 3>(0, 3 * pid).inverse().eval();
      }
    }

    double energy = 0;
    for (int pid = 0; pid < mesh.num_polys(); ++pid)
      energy += this->eval(mesh, options, x, pid);

    return energy;
  }
};

template <typename Mesh, typename DeformE, typename FacetE>
struct MeshDeformFunctor : public Functor<double, Eigen::Dynamic, 1> {
  MeshDeformFunctor(Mesh &mesh, DeformE &deformE, FacetE &facetE)
      : _mesh(mesh), Functor<double, Eigen::Dynamic, 1>(mesh.num_verts() * 3,
                                                        1),
        _deformE(deformE), _facetE(facetE) {}

  HEXER_INLINE double evalOnePoly(const Eigen::VectorXd &x, int pid) {
    double normal_e = 0.0, deform_e = 0.0;
    if (_mesh.poly_data(pid).flags[0])
      normal_e = _facetE.execute(x, pid);
    deform_e = _deformE.execute(x, pid);
    // return deform_e +
    //        std::min(std::max(normal_e / deform_e, 1e3), 1e16) * normal_e;
    return normal_e + deform_e;
  }

  int operator()(const Eigen::VectorXd &x, Eigen::Vector<double, 1> &fvec) {
    fvec[0] = 0;
    for (int pid = 0; pid < _mesh.num_polys(); ++pid)
      fvec[0] += evalOnePoly(x, pid);

    return 0;
  }

  int f_evalOne(const Eigen::VectorXd &x, Eigen::Vector<double, 1> &fvec,
                int vid) {
    fvec[0] = 0;
    for (auto &pid : _mesh.adj_v2p(vid))
      fvec[0] += evalOnePoly(x, pid);

    return 0;
  }

  int df(const Eigen::VectorXd &x, Eigen::VectorXd &jac) {
    return NumericalDiff<DiffMode::Central>(*this, x, jac);
  }

  Mesh &_mesh;
  DeformE &_deformE;
  FacetE &_facetE;
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class PolyCubeGen
    : public CrtpExprBase<device, PolyCubeGen<device, ParamTuple>, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
            DeformEnergyOptions d_options, FacetNormalDeformOption f_options) {
    auto deform_op = DeformEnergy()(mesh, d_options);
    auto facet_op = FacetNormalsEnergy()(mesh, f_options);
    auto functor = MeshDeformFunctor(mesh, deform_op, facet_op);
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(
        mesh.vector_verts().data()->ptr(), mesh.num_verts() * 3);

    PolyhedralSurfMarker()(mesh).execute();
    deform_op.execute(x);
    facet_op.execute(x);

    // Eigen::LevenbergMarquardt<decltype(functor)> solver(functor);
    // auto status = solver.minimizeInit(x);
    // do {
    //   status = solver.minimizeOneStep(x);
    //   spdlog::info("iter: {} | TargetVal: {:03.2f}", solver.iter,
    //                solver.fvec.sum());
    // } while (status == Eigen::LevenbergMarquardtSpace::Running);

    BFGS<decltype(functor)> solver(functor);
    solver.solve(x);
    for (int i = 0; i < mesh.num_verts(); ++i) {
      mesh.vert(i)[0] = x[3 * i];
      mesh.vert(i)[1] = x[3 * i + 1];
      mesh.vert(i)[2] = x[3 * i + 2];
    }
  }
};

} // namespace Hexer