#pragma once
#include <cinolib/meshes/meshes.h>

#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <numeric>
#include <range/v3/all.hpp>

#include "bfgs.h"
#include "cinolib_enhance.h"
#include "expr.h"
#include "graph.h"
#include "mesh.h"

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

    for (int i = 0; i < nv; ++i) offset.push_back(i * mesh.num_verts());

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
    for (int i = 0; i < nv; ++i) offset.push_back(i * mesh.num_verts());

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
  HEXER_INLINE auto eval(
      const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
      FacetNormalDeformOption options, const Eigen::VectorXd &x, int vid,
      bool gradient) {
    double n_gsn = 0.0;
    Eigen::Vector3d d_gsn(0, 0, 0);
    for (auto fid : mesh.adj_v2f(vid)) {
      if (mesh.face_data(fid).flags[cinolib::UNUSED_0]) {
        auto &adj_vid = mesh.adj_f2v(fid);
        auto offset = vert_offset_within_tri(mesh, fid, vid);
        auto v0 = x.block<3, 1>(adj_vid[offset] * 3, 0);
        auto v1 = x.block<3, 1>(adj_vid[(offset + 1) % 3] * 3, 0);
        auto v2 = x.block<3, 1>(adj_vid[(offset + 2) % 3] * 3, 0);

        Eigen::Vector3d vs1 = v1 - v0;
        Eigen::Vector3d vs2 = v2 - v0;
        Eigen::Vector3d cross_v = vs1.cross(vs2);
        double cross_v_norm2 = 1.0 / cross_v.norm();
        Eigen::Vector3d cal_gsn =
            cross_v * cross_v_norm2 - _gsn.col(_face_map[fid]);
        n_gsn += cal_gsn.squaredNorm();

        if (gradient) {
          Eigen::Matrix3d d_vs;

          Eigen::Vector3d d_vs_x(0, vs1[2] - vs2[2], vs2[1] - vs1[1]);
          double d_vd_x =
              cross_v_norm2 * (vs[1] * d_vs_x[1] + vs[2] * d_vs_x[2]);
          d_vs_x = cross_v_norm2 * d_vs_x -
                   cross_v_norm2 * cross_v_norm2 * d_vd_x * cross_v;

          Eigen::Vector3d d_vs_y(vs2[2] - vs1[2], 0, vs1[0] - vs2[0]);
          double d_vd_y =
              cross_v_norm2 * (vs[0] * d_vs_y[0] + vs[2] * d_vs_y[2]);
          d_vs_y = cross_v_norm2 * d_vs_y -
                   cross_v_norm2 * cross_v_norm2 * d_vd_y * cross_v;

          Eigen::Vector3d d_vs_z(vs1[1] - vs2[1], vs2[0] - vs1[0], 0);
          double d_vd_z =
              cross_v_norm2 * (vs[0] * d_vs_z[0] + vs[1] * d_vs_z[1]);
          d_vs_z = cross_v_norm2 * d_vs_z -
                   cross_v_norm2 * cross_v_norm2 * d_vd_z * cross_v;

          d_vs.row(0) = d_vs_x.transpose();
          d_vs.row(1) = d_vs_y.transpose();
          d_vs.row(2) = d_vs_z.transpose();

          d_gsn = 2 * d_vs * cal_gsn;
        }
      }
    }

    return std::make_pair(n_gsn, d_gsn);
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
  Eigen::Matrix3Xd _A0;

  template <typename VEC>
  HEXER_INLINE auto poly_affine(const VEC &x, int v0, int v1, int v2, int v3) {
    Eigen::Matrix3d A;
    A.col(0) = x.block<3, 1>(3 * v0, 0) - x.block<3, 1>(3 * v1, 0);
    A.col(1) = x.block<3, 1>(3 * v0, 0) - x.block<3, 1>(3 * v2, 0);
    A.col(2) = x.block<3, 1>(3 * v0, 0) - x.block<3, 1>(3 * v3, 0);
    return A;
  }

 public:
  template <typename... Args>
  DeformEnergy(Args &&...args)
      : CrtpExprBase<device, DeformEnergy<device, ParamTuple>, ParamTuple>(
            std::forward<decltype(args)>(args)...) {}

  template <typename M, typename V, typename E, typename F, typename P>
  HEXER_INLINE auto eval(
      const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
      DeformEnergyOptions options, const Eigen::VectorXd &x) {
    if (_A0.size() == 0) {
      _A0.resize(3, mesh.num_polys() * 12);
      auto x = Eigen::Map<Eigen::VectorXd>(
          const_cast<cinolib::AbstractPolyhedralMesh<M, V, E, F, P> *>(&mesh)
              ->vector_verts()
              .data()
              ->ptr(),
          mesh.num_verts() * 3);
      for (int pid = 0; pid < mesh.num_polys(); ++pid) {
        for (auto &&[off, vid] : mesh.adj_p2v(pid) | ranges::views::enumerate)
          _A0.block<3, 3>(0, pid * 12 + off * 3) =
              poly_affine(x, vid, mesh.adj_p2v(pid)[(off + 1) % 4],
                          mesh.adj_p2v(pid)[(off + 2) % 4],
                          mesh.adj_p2v(pid)[(off + 3) % 4])
                  .inverse();
      }
    }
  }

  template <typename M, typename V, typename E, typename F, typename P>
  HEXER_INLINE auto eval(
      const cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
      DeformEnergyOptions options, const Eigen::VectorXd &x, int vid,
      bool gradient) {
    double energy = 0.0;
    Eigen::Vector3d d_energy(0, 0, 0);

    for (auto pid : mesh.adj_v2p(vid)) {
      int off = vert_offset_within_tet(mesh, pid, vid);
      auto A_0 = _A0.block<3, 3>(0, pid * 12 + off * 3);
      auto A_1 = poly_affine(x, vid, mesh.adj_p2v(pid)[(off + 1) % 4],
                             mesh.adj_p2v(pid)[(off + 2) % 4],
                             mesh.adj_p2v(pid)[(off + 3) % 4]);
      auto A_expr = A_1 * A_0;
      Eigen::Matrix3d A_inv = A_expr.inverse();
      double F2_A = A_expr.squaredNorm();
      double F2_A_inv = A_inv.squaredNorm();

      double conformal = 0.125 * F2_A * F2_A_inv - 0.125;
      double A_det = A_expr.determinant();
      double volumetric = 0.5 * (A_det + 1.0 / A_det);
      energy += std::exp(options.s * (options.alpha * conformal +
                                      (1 - options.alpha) * volumetric));

      if (gradient) {
        auto g_prefix =
            0.25 * (F2_A_inv * A_expr -
                    F2_A * A_inv.transpose() * A_inv * A_inv.transpose());
        Eigen::Matrix3d d_A;
        d_A.row(0) = Eigen::Vector3d(A_0.col(0).sum(), A_0.col(1).sum(),
                                     A_0.col(2).sum())
                         .transpose();
        d_A.row(1) = d_A.row(0);
        d_A.row(2) = d_A.row(1);

        d_energy +=
            d_A.cwiseProduct(g_prefix).rowwise().sum() +
            0.5 * (A_det * A_inv.transpose()).cwiseProduct(d_A).rowwise().sum();
      }
    }
    return std::make_pair(energy, d_energy);
  }
};

template <typename Mesh, typename DeformE, typename FacetE>
struct MeshDeformFunctor : public Functor<double, Eigen::Dynamic, 1> {
  MeshDeformFunctor(Mesh &mesh, DeformE &deformE, FacetE &facetE, int start_id,
                    int end_id)
      : _mesh(mesh),
        Functor<double, Eigen::Dynamic, 1>(mesh.num_verts() * 3, 1),
        _deformE(deformE),
        _facetE(facetE),
        _sid(start_id),
        _eid(end_id) {}

  HEXER_INLINE auto evalOneVertex(const Eigen::VectorXd &x, int vid,
                                  bool gradient) {
    // auto [normal_e, d_normal_e] = _deformE.execute(x, vid, gradient);
    auto [facet_e, d_facet_e] = _facetE.execute(x, vid, gradient);
    return std::make_pair(facet_e, (d_facet_e).eval());
  }

  int operator()(const Eigen::VectorXd &x, Eigen::Vector<double, 1> &fvec,
                 Eigen::VectorXd &fjac, bool gradient) {
    fvec[0] = 0;
    for (int vid = _sid; vid < _eid; ++vid) {
      auto [fval, d_fval] = evalOneVertex(x, vid, gradient);
      fvec[0] += fval;
      if (gradient) fjac.block<3, 1>(vid * 3, 0) = d_fval;
    }
    return 0;
  }

  Mesh &_mesh;
  DeformE &_deformE;
  FacetE &_facetE;
  int _sid;
  int _eid;
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class PolyCubeGen
    : public CrtpExprBase<device, PolyCubeGen<device, ParamTuple>, ParamTuple> {
 public:
  template <typename M, typename V, typename E, typename F, typename P>
  auto eval(cinolib::AbstractPolyhedralMesh<M, V, E, F, P> &mesh,
            DeformEnergyOptions d_options, FacetNormalDeformOption f_options,
            int iter_n) {
    // rerange mesh index by color group, vertices belong to one group are
    // placed togather
    auto [_mesh, groups] = VertexColoring()(mesh, SortOrder::AscendOrder) |
                           GraphColorMap() | RerangeVertexByColor()(mesh) |
                           evalOp();
    // after rerange the index, mark the surface vertices flags
    PolyhedralSurfMarker()(_mesh).execute();

    // construct deform operators for mesh, the deform operator is irrelevant
    // with color group
    auto deform_op = DeformEnergy()(_mesh, d_options);
    auto facet_op = FacetNormalsEnergy()(_mesh, f_options);
    using SolverFunctorType =
        decltype(MeshDeformFunctor(_mesh, deform_op, facet_op, 0, 0));
    std::vector<BFGS<SolverFunctorType>> solver_list;

    // construct solver for each color group
    for (auto [i, val] : groups | ranges::views::enumerate)
      solver_list.emplace_back(MeshDeformFunctor(_mesh, deform_op, facet_op,
                                                 i ? groups[i - 1] : 0, val));

    for (int _iter = 0; _iter < iter_n; ++_iter) {
      // optimize only one color group once
      for (auto [i, val] : groups | ranges::views::enumerate) {
        int sid = i ? groups[i - 1] : 0;
        auto &solver = solver_list[i];
        auto x_partial = Eigen::Map<Eigen::VectorXd>(
            _mesh.vector_verts().data()->ptr() + sid * 3, val * 3);
        solver.solve(x_partial);
      }
      spdlog::info("iter: {} finished.", _iter);
    }

    // auto deform_op = DeformEnergy()(mesh, d_options);
    // auto facet_op = FacetNormalsEnergy()(mesh, f_options);
    // auto functor = MeshDeformFunctor(mesh, deform_op, facet_op);
    // Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(
    //     mesh.vector_verts().data()->ptr(), mesh.num_verts() * 3);

    // PolyhedralSurfMarker()(mesh).execute();
    // deform_op.execute(x);
    // facet_op.execute(x);

    // BFGS<decltype(functor)> solver(functor);
    // solver.solve(x);

    // VertexUpdate().execute(mesh, x);
  }
};

}  // namespace Hexer