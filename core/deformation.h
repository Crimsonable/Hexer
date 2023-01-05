#pragma once
#include "bfgs.h"
#include "expr.h"
#include "mesh.h"

#include <cinolib/meshes/meshes.h>

namespace Hexer {
enum class SmoothMethod { UNIFORM, COTANGENT };

struct LaplacianOptions {
  uint n_smooth = 1;
  double lambda = 0.001;
  SmoothMethod method = SmoothMethod::UNIFORM;
};

class UniformWeighter {
public:
  template <typename M, typename V, typename E, typename P>
  void operator()(std::vector<Eigen::Triplet<double>> &entries, uint nv,
                  uint current_vid,
                  const cinolib::AbstractMesh<M, V, E, P> &mesh) {
    auto ring = mesh.vert_n_ring(current_vid, 1);
    std::vector<uint> offset;

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
  void operator()(std::vector<Eigen::Triplet<double>> &entries, uint nv,
                  uint current_vid, const cinolib::Trimesh<M, V, E, P> &mesh) {
    std::vector<uint> offset;
    for (int i = 0; i < nv; ++i)
      offset.push_back(i * mesh.num_verts());

    // caculate cotangent weight using cinolib internal function
    std::vector<std::pair<uint, double>> weight;
    mesh.vert_weights_cotangent(current_vid, weight);

    double sum =
        std::accumulate(weight.begin(), weight.end(), 0.0,
                        [](double w1, const std::pair<uint, double> &w) {
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
  void operator()(std::vector<Eigen::Triplet<double>> &entries, uint nv,
                  uint current_vid, const cinolib::Trimesh<M, V, E, P> &mesh) {}
};

// caculate Laplacian matrix using given method(cotangent,uniform)
template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class LaplacianMatrix
    : public CrtpExprBase<Device::CPU, LaplacianMatrix, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const LaplacianOptions &options,
                   cinolib::AbstractMesh<M, V, E, P> &mesh) {
    uint n = mesh.num_verts();
    auto Laplacian = Eigen::SparseMatrix<double>(n * 3, n * 3);
    std::vector<Eigen::Triplet<double>> entries;

    if (options.method == SmoothMethod::UNIFORM) {
      auto weighter = UniformWeighter();
      for (uint vid = 0; vid < mesh.num_verts(); ++vid) {
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
      for (uint vid = 0; vid < tri_mesh->num_verts(); ++vid) {
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
    : public CrtpExprBase<device, LaplacianSmoother, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const LaplacianOptions &options,
                   Eigen::SparseMatrix<double> &&Laplacian,
                   cinolib::AbstractMesh<M, V, E, P> &mesh) {
    std::vector<double> vertices_coords;
    uint n = mesh.num_verts();
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

struct DeformationOptions {
  double sigma = 0.01;
};

template <typename M = cinolib::Mesh_std_attributes,
          typename V = cinolib::Vert_std_attributes,
          typename E = cinolib::Edge_std_attributes,
          typename Pf = cinolib::Polygon_std_attributes,
          typename Pv = cinolib::Polyhedron_std_attributes>
struct NormalSmoothFunctor {};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GaussianSmoothFacetNormals
    : public CrtpExprBase<device, GaussianSmoothFacetNormals, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const DeformationOptions &options,
                   cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
    auto ns = mesh.vector_poly_normals();
    Eigen::Matrix3Xd normals =
        Eigen::Map<Eigen::Matrix3Xd>(ns.data(), 3, ns.size());

    auto vs = mesh.verts();
    Eigen::Matrix3Xd vertices =
        Eigen::Map<Eigen::Matrix3Xd>(vs.data(), 3, vs.size());
    Eigen::Matrix3Xd poly_center;
    poly_center.resize(3, mesh.num_polys());

    Eigen::VectorXd area;
    area.resize(mesh.num_polys());
    for (int fid = 0; fid < mesh.num_polys(); ++fid)
      area.coeffRef(fid) = mesh.poly_area(fid);

    cinolib::vec3d vp{0, 0, 0};
    for (int fid = 0; fid < mesh.num_polys(); ++fid) {
      auto vf = mesh.poly_verts(fid);
      for (int i = 0; i < vf.size(); ++i)
        vp = vp + vf[i];
      vp = 1.0 / vf.size() * vp;

      poly_center.coeffRef(0, fid) = vp[0];
      poly_center.coeffRef(1, fid) = vp[1];
      poly_center.coeffRef(2, fid) = vp[2];
    }

    Eigen::Matrix3Xd gsn;
    gsn.resize(3, mesh.num_polys());

    for (int fid = 0; fid < mesh.num_poys(); ++fid) {
      gsn.col(fid) = normals.rowwise()
                         .cwiseProduct(area.transpose().cwiseProduct(Eigen::exp(
                             (-1.0 / std::pow(options.sigma, 2)) *
                             (poly_center.colwise() - poly_center.col(fid))
                                 .colwise()
                                 .norm())))
                         .rowwise()
                         .sum();
    }
    return gsn;
  }
};

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class NormalSmooothEnergy
    : public CrtpExprBase<device, NormalSmooothEnergy, ParamTuple> {

  template <typename M, typename V, typename E, typename P>
  static auto eval(cinolib::AbstractPolygonMesh<M, V, E, P> &surface,
                   Eigen::Matrix3Xd &vertices, Eigen::Matrix3Xd &centeral_point,
                   Eigen::Matrix3Xd &normals, Eigen::VectorXd &areas,
                   Eigen::VectorXd &ns_energy) {
    for (int i = 0; i < centeral_point.size(); ++i) {
      double exp_wgt =
          Eigen::exp((centeral_point.colwise() - centeral_point.col(i))
                         .rowwise()
                         .norm())
              .cwiseProduct(areas);
    }
  }
};
} // namespace Hexer