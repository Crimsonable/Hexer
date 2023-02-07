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

template <typename M, typename V, typename E, typename P>
inline auto poly_centroid(cinolib::AbstractPolygonMesh<M, V, E, P> &mesh,
                          uint fid) {
  auto tmp = mesh.poly_centroid(fid);
  Eigen::Vector3d p;

  p(0) = tmp[0];
  p(1) = tmp[1];
  p(2) = tmp[2];

  return p;
}

template <Device device = Device::CPU, typename ParamTuple = std::tuple<>>
class GaussianDistanceWeight
    : public CrtpExprBase<device, GaussianDistanceWeight, ParamTuple> {
public:
  auto eval(const Eigen::Matrix3Xd &normals, const Eigen::Matrix3Xd &centers,
            const Eigen::VectorXd &areas, double sigma, uint fid) {
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
    : public CrtpExprBase<device, GaussianSmoothFacetNormals, ParamTuple> {
public:
  template <typename M, typename V, typename E, typename P>
  static auto eval(const DeformationOptions &options,
                   cinolib::AbstractPolygonMesh<M, V, E, P> &mesh) {
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

  for (uint fid = 0; fid < mesh.num_polys(); ++fid) {
    cinolib::vec3d n{0, 0, 0};
    Eigen::Vector3d p = poly_centroid(mesh, fid);
    for (uint fid2 = 0; fid2 < mesh.num_polys(); ++fid2) {
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
class NormalSmooothEnergy
    : public CrtpExprBase<device, NormalSmooothEnergy, ParamTuple> {

  template <typename M, typename V, typename E, typename P>
  static auto eval(cinolib::AbstractPolygonMesh<M, V, E, P> &surface,
                   Eigen::Matrix3Xd &vertices, Eigen::Matrix3Xd &centeral_point,
                   Eigen::Matrix3Xd &normals, Eigen::VectorXd &areas,
                   Eigen::VectorXd &ns_energy) {}
};
} // namespace Hexer