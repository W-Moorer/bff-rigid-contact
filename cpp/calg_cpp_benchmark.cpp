#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat32 = Eigen::Matrix<double, 3, 2>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

constexpr double EPS = 1.0e-12;
constexpr double PI = 3.141592653589793238462643383279502884;
static double g_guide_target_gap = 0.0;
static double g_guide_mu = 0.40;

static double clamp(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

static Vec3 normalize(const Vec3& v, const Vec3& fallback = Vec3(0.0, 0.0, 1.0)) {
    const double n = v.norm();
    return n > 1.0e-14 ? v / n : fallback;
}

static Mat3 rotation_from_rotvec(const Vec3& w) {
    const double theta = w.norm();
    Mat3 I = Mat3::Identity();
    Mat3 K;
    if (theta <= 1.0e-14) {
        K << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
        return I + K;
    }
    Vec3 a = w / theta;
    K << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return I + std::sin(theta) * K + (1.0 - std::cos(theta)) * (K * K);
}

static Mat3 orthonormalize(const Mat3& R) {
    Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 out = svd.matrixU() * svd.matrixV().transpose();
    if (out.determinant() < 0.0) {
        Mat3 U = svd.matrixU();
        U.col(2) *= -1.0;
        out = U * svd.matrixV().transpose();
    }
    return out;
}

struct RigidConfiguration {
    Vec3 position = Vec3::Zero();
    Mat3 rotation = Mat3::Identity();
};

static RigidConfiguration make_configuration(const Vec3& position, const Mat3& rotation = Mat3::Identity()) {
    return {position, rotation};
}

struct AABB {
    Vec3 lo = Vec3::Zero();
    Vec3 hi = Vec3::Zero();
    bool intersects(const AABB& o) const {
        return (hi.array() >= o.lo.array()).all() && (o.hi.array() >= lo.array()).all();
    }
};

static AABB aabb_from_points(const std::array<Vec3, 3>& pts, double margin) {
    AABB b;
    b.lo = pts[0];
    b.hi = pts[0];
    for (const auto& p : pts) {
        b.lo = b.lo.cwiseMin(p);
        b.hi = b.hi.cwiseMax(p);
    }
    b.lo.array() -= margin;
    b.hi.array() += margin;
    return b;
}

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<std::array<int, 3>> faces;
    std::string name;
};

static Mesh transform_mesh(const Mesh& local, const RigidConfiguration& config, const std::string& name = "") {
    Mesh out;
    out.name = name.empty() ? local.name : name;
    out.vertices.reserve(local.vertices.size());
    out.normals.reserve(local.normals.size());
    for (const Vec3& v : local.vertices) out.vertices.push_back(config.rotation * v + config.position);
    for (const Vec3& n : local.normals) out.normals.push_back(normalize(config.rotation * n));
    out.faces = local.faces;
    return out;
}

static double mesh_bounding_radius(const Mesh& local) {
    double r = 0.0;
    for (const Vec3& v : local.vertices) r = std::max(r, v.norm());
    return r;
}

struct Primitive {
    std::string mesh_name;
    int face_index = 0;
    std::array<Vec3, 3> v;
    std::array<Vec3, 3> n;
    double error_bound = 0.0;
    AABB aabb;

    Vec3 centroid() const { return (v[0] + v[1] + v[2]) / 3.0; }
    Vec3 face_normal() const {
        Vec3 fallback = normalize((n[0] + n[1] + n[2]) / 3.0);
        return normalize((v[1] - v[0]).cross(v[2] - v[0]), fallback);
    }
};

static double angle_between(const Vec3& a, const Vec3& b) {
    const Vec3 na = normalize(a);
    const Vec3 nb = normalize(b);
    return std::acos(clamp(na.dot(nb), -1.0, 1.0));
}

static std::vector<Primitive> build_primitives(const Mesh& mesh, double contact_margin) {
    std::vector<Primitive> out;
    out.reserve(mesh.faces.size());
    for (int fi = 0; fi < static_cast<int>(mesh.faces.size()); ++fi) {
        Primitive p;
        p.mesh_name = mesh.name;
        p.face_index = fi;
        auto f = mesh.faces[fi];
        for (int k = 0; k < 3; ++k) {
            p.v[k] = mesh.vertices[f[k]];
            p.n[k] = normalize(mesh.normals[f[k]]);
        }
        double kappa = 0.0;
        for (auto e : {std::array<int, 2>{0, 1}, std::array<int, 2>{1, 2}, std::array<int, 2>{2, 0}}) {
            double len = (p.v[e[0]] - p.v[e[1]]).norm();
            if (len > 1.0e-12) kappa = std::max(kappa, angle_between(p.n[e[0]], p.n[e[1]]) / len);
        }
        double h = std::max({(p.v[1] - p.v[0]).norm(), (p.v[2] - p.v[1]).norm(), (p.v[0] - p.v[2]).norm()});
        p.error_bound = 1.5 * kappa * h * h / 8.0;
        p.aabb = aabb_from_points(p.v, contact_margin + p.error_bound);
        out.push_back(p);
    }
    return out;
}

struct ClosestPair {
    Vec3 a = Vec3::Zero();
    Vec3 b = Vec3::Zero();
    double distance = std::numeric_limits<double>::infinity();
};

static std::pair<Vec3, Eigen::Vector3d> closest_point_on_triangle(const Vec3& p, const std::array<Vec3, 3>& tri) {
    const Vec3& a = tri[0];
    const Vec3& b = tri[1];
    const Vec3& c = tri[2];
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;
    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return {a, Eigen::Vector3d(1.0, 0.0, 0.0)};
    Vec3 bp = p - b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return {b, Eigen::Vector3d(0.0, 1.0, 0.0)};
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        return {a + v * ab, Eigen::Vector3d(1.0 - v, v, 0.0)};
    }
    Vec3 cp = p - c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return {c, Eigen::Vector3d(0.0, 0.0, 1.0)};
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        double w = d2 / (d2 - d6);
        return {a + w * ac, Eigen::Vector3d(1.0 - w, 0.0, w)};
    }
    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return {b + w * (c - b), Eigen::Vector3d(0.0, 1.0 - w, w)};
    }
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    double u = 1.0 - v - w;
    return {u * a + v * b + w * c, Eigen::Vector3d(u, v, w)};
}

static std::tuple<Vec3, Vec3, double, double> closest_points_on_segments(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
    Vec3 d1 = q1 - p1;
    Vec3 d2 = q2 - p2;
    Vec3 r = p1 - p2;
    double a = d1.dot(d1);
    double e = d2.dot(d2);
    double f = d2.dot(r);
    double s = 0.0;
    double t = 0.0;
    if (a <= 1e-15 && e <= 1e-15) return {p1, p2, 0.0, 0.0};
    if (a <= 1e-15) {
        t = clamp(f / e, 0.0, 1.0);
    } else {
        double c = d1.dot(r);
        if (e <= 1e-15) {
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            double b = d1.dot(d2);
            double denom = a * e - b * b;
            s = denom != 0.0 ? clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            double tnom = b * s + f;
            if (tnom < 0.0) {
                t = 0.0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (tnom > e) {
                t = 1.0;
                s = clamp((b - c) / a, 0.0, 1.0);
            } else {
                t = tnom / e;
            }
        }
    }
    return {p1 + d1 * s, p2 + d2 * t, s, t};
}

static ClosestPair triangle_triangle_closest(const std::array<Vec3, 3>& ta, const std::array<Vec3, 3>& tb) {
    ClosestPair best;
    auto update = [&](const Vec3& a, const Vec3& b) {
        double d = (a - b).norm();
        if (d < best.distance) best = {a, b, d};
    };
    for (int i = 0; i < 3; ++i) update(ta[i], closest_point_on_triangle(ta[i], tb).first);
    for (int i = 0; i < 3; ++i) update(closest_point_on_triangle(tb[i], ta).first, tb[i]);
    for (auto ea : {std::array<int, 2>{0, 1}, std::array<int, 2>{1, 2}, std::array<int, 2>{2, 0}}) {
        for (auto eb : {std::array<int, 2>{0, 1}, std::array<int, 2>{1, 2}, std::array<int, 2>{2, 0}}) {
            auto [a, b, s, t] = closest_points_on_segments(ta[ea[0]], ta[ea[1]], tb[eb[0]], tb[eb[1]]);
            update(a, b);
        }
    }
    return best;
}

static Eigen::Matrix<double, 6, 1> basis6(const Vec2& xi) {
    double x = xi.x(), y = xi.y();
    Eigen::Matrix<double, 6, 1> b;
    b << 1.0, x, y, x * x, x * y, y * y;
    return b;
}

static Eigen::Matrix<double, 6, 1> basis_dx(const Vec2& xi) {
    double x = xi.x(), y = xi.y();
    Eigen::Matrix<double, 6, 1> b;
    b << 0.0, 1.0, 0.0, 2.0 * x, y, 0.0;
    return b;
}

static Eigen::Matrix<double, 6, 1> basis_dy(const Vec2& xi) {
    double x = xi.x(), y = xi.y();
    Eigen::Matrix<double, 6, 1> b;
    b << 0.0, 0.0, 1.0, 0.0, x, 2.0 * y;
    return b;
}

static Eigen::Vector3d barycentric_2d(const Vec2& p, const std::array<Vec2, 3>& tri) {
    double ax = tri[0].x(), ay = tri[0].y();
    double bx = tri[1].x(), by = tri[1].y();
    double cx = tri[2].x(), cy = tri[2].y();
    double v0x = bx - ax, v0y = by - ay;
    double v1x = cx - ax, v1y = cy - ay;
    double v2x = p.x() - ax, v2y = p.y() - ay;
    double denom = v0x * v1y - v1x * v0y;
    if (std::abs(denom) <= EPS) return Eigen::Vector3d(1.0, 0.0, 0.0);
    double b1 = (v2x * v1y - v1x * v2y) / denom;
    double b2 = (v0x * v2y - v2x * v0y) / denom;
    return Eigen::Vector3d(1.0 - b1 - b2, b1, b2);
}

static Vec2 point_from_bary2d(const std::array<Vec2, 3>& tri, const Eigen::Vector3d& b) {
    return b[0] * tri[0] + b[1] * tri[1] + b[2] * tri[2];
}

static Vec2 clamp_to_triangle_2d(const Vec2& p, const std::array<Vec2, 3>& tri) {
    Eigen::Vector3d b = barycentric_2d(p, tri);
    if ((b.array() >= -1.0e-12).all()) {
        if ((b.array() < 0.0).any()) {
            Eigen::Vector3d bc = b.cwiseMax(0.0);
            double s = std::max(bc.sum(), EPS);
            return point_from_bary2d(tri, bc / s);
        }
        return p;
    }
    Vec2 best = tri[0];
    double best_d = std::numeric_limits<double>::infinity();
    for (auto e : {std::array<int, 2>{0, 1}, std::array<int, 2>{1, 2}, std::array<int, 2>{2, 0}}) {
        Vec2 a = tri[e[0]], c = tri[e[1]];
        Vec2 edge = c - a;
        double denom = edge.dot(edge);
        Vec2 q = denom <= EPS ? a : a + clamp((p - a).dot(edge) / denom, 0.0, 1.0) * edge;
        double d = (p - q).norm();
        if (d < best_d) {
            best_d = d;
            best = q;
        }
    }
    return best;
}

struct Patch {
    Vec3 origin = Vec3::Zero();
    Vec3 t1 = Vec3::UnitX();
    Vec3 t2 = Vec3::UnitY();
    Vec3 n0 = Vec3::UnitZ();
    Vec6 coeff = Vec6::Zero();
    std::array<Vec2, 3> domain;
    double error_bound = 0.0;
    bool second_cached = false;
    std::array<Vec3, 3> second;

    static Patch from_primitive(const Primitive& p) {
        Patch out;
        out.origin = p.v[0];
        Vec3 mean_n = normalize((p.n[0] + p.n[1] + p.n[2]) / 3.0);
        out.n0 = normalize(p.face_normal(), mean_n);
        Vec3 e01 = p.v[1] - p.v[0];
        if (e01.norm() <= EPS) {
            out.t1 = Vec3::UnitX();
            out.t2 = normalize(out.n0.cross(out.t1), Vec3::UnitY());
            out.t1 = normalize(out.t2.cross(out.n0), Vec3::UnitX());
        } else {
            out.t1 = normalize(e01);
            out.t1 = normalize(out.t1 - out.t1.dot(out.n0) * out.n0, out.t1);
            out.t2 = normalize(out.n0.cross(out.t1), Vec3::UnitY());
            out.t1 = normalize(out.t2.cross(out.n0), out.t1);
        }
        std::array<Vec2, 3> xis;
        std::array<double, 3> hs;
        for (int i = 0; i < 3; ++i) {
            Vec3 r = p.v[i] - out.origin;
            xis[i] = Vec2(out.t1.dot(r), out.t2.dot(r));
            hs[i] = out.n0.dot(r);
        }
        out.domain = xis;
        Eigen::Matrix<double, 9, 6> A;
        Eigen::Matrix<double, 9, 1> rhs;
        int row = 0;
        for (int i = 0; i < 3; ++i) {
            A.row(row) = 20.0 * basis6(xis[i]).transpose();
            rhs[row++] = 20.0 * hs[i];
        }
        for (int i = 0; i < 3; ++i) {
            Vec3 n = normalize(p.n[i], out.n0);
            if (n.dot(out.n0) < 0.0) n = -n;
            double nd = n.dot(out.n0);
            double gx = std::abs(nd) < 1.0e-8 ? 0.0 : -n.dot(out.t1) / nd;
            double gy = std::abs(nd) < 1.0e-8 ? 0.0 : -n.dot(out.t2) / nd;
            A.row(row) = basis_dx(xis[i]).transpose();
            rhs[row++] = gx;
            A.row(row) = basis_dy(xis[i]).transpose();
            rhs[row++] = gy;
        }
        out.coeff = A.colPivHouseholderQr().solve(rhs);
        out.error_bound = p.error_bound;
        return out;
    }

    double height(const Vec2& xi) const { return basis6(xi).dot(coeff); }
    Vec2 grad_height(const Vec2& xi) const { return Vec2(basis_dx(xi).dot(coeff), basis_dy(xi).dot(coeff)); }
    Vec3 eval(const Vec2& xi) const { return origin + xi.x() * t1 + xi.y() * t2 + height(xi) * n0; }
    Mat32 derivatives(const Vec2& xi) const {
        Vec2 g = grad_height(xi);
        Mat32 J;
        J.col(0) = t1 + g.x() * n0;
        J.col(1) = t2 + g.y() * n0;
        return J;
    }
    const std::array<Vec3, 3>& second_derivatives() {
        if (!second_cached) {
            second[0] = (2.0 * coeff[3]) * n0;
            second[1] = coeff[4] * n0;
            second[2] = (2.0 * coeff[5]) * n0;
            second_cached = true;
        }
        return second;
    }
    Vec2 clamp_xi(const Vec2& xi) const { return clamp_to_triangle_2d(xi, domain); }
    Vec2 project_to_domain(const Vec3& x) const {
        Vec3 r = x - origin;
        return clamp_xi(Vec2(t1.dot(r), t2.dot(r)));
    }
    Vec3 normal(const Vec2& xi) const {
        Mat32 J = derivatives(xi);
        return normalize(J.col(0).cross(J.col(1)), n0);
    }
    Eigen::Vector3d barycentric(const Vec2& xi) const {
        Eigen::Vector3d b = barycentric_2d(xi, domain);
        Eigen::Vector3d bc = b.cwiseMax(0.0);
        double s = bc.sum();
        return s > EPS ? bc / s : Eigen::Vector3d(1.0, 0.0, 0.0);
    }
};

struct CurvedResult {
    bool valid = false;
    bool contact = false;
    Vec3 point_a = Vec3::Zero();
    Vec3 point_b = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
    double gap = 0.0;
    Eigen::Vector3d bary_a = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d bary_b = Eigen::Vector3d(1.0, 0.0, 0.0);
    int iterations = 0;
    double residual = 0.0;
};

struct Objective {
    double E = 0.0;
    Eigen::Matrix<double, 4, 1> grad = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
    Vec2 xia = Vec2::Zero();
    Vec2 xib = Vec2::Zero();
    Vec3 Xa = Vec3::Zero();
    Vec3 Xb = Vec3::Zero();
    double residual = 0.0;
};

static Objective objective_face_face(Patch& a, Patch& b, const Vec2& za, const Vec2& zb) {
    Objective o;
    o.xia = a.clamp_xi(za);
    o.xib = b.clamp_xi(zb);
    Mat32 Ja = a.derivatives(o.xia);
    Mat32 Jb = b.derivatives(o.xib);
    auto Sa = a.second_derivatives();
    auto Sb = b.second_derivatives();
    o.Xa = a.eval(o.xia);
    o.Xb = b.eval(o.xib);
    Vec3 r = o.Xa - o.Xb;
    o.E = 0.5 * r.dot(r);
    o.grad.segment<2>(0) = Ja.transpose() * r;
    o.grad.segment<2>(2) = -Jb.transpose() * r;
    Eigen::Matrix2d Haa = Ja.transpose() * Ja;
    Eigen::Matrix2d Hbb = Jb.transpose() * Jb;
    Haa(0, 0) += r.dot(Sa[0]);
    Haa(0, 1) += r.dot(Sa[1]);
    Haa(1, 0) += r.dot(Sa[1]);
    Haa(1, 1) += r.dot(Sa[2]);
    Hbb(0, 0) -= r.dot(Sb[0]);
    Hbb(0, 1) -= r.dot(Sb[1]);
    Hbb(1, 0) -= r.dot(Sb[1]);
    Hbb(1, 1) -= r.dot(Sb[2]);
    Eigen::Matrix2d Hab = -Ja.transpose() * Jb;
    o.H.block<2, 2>(0, 0) = Haa;
    o.H.block<2, 2>(2, 2) = Hbb;
    o.H.block<2, 2>(0, 2) = Hab;
    o.H.block<2, 2>(2, 0) = Hab.transpose();
    o.residual = o.grad.norm();
    return o;
}

static CurvedResult solve_curved_pair(const Primitive& pa, const Primitive& pb, double d_hat) {
    Patch a = Patch::from_primitive(pa);
    Patch b = Patch::from_primitive(pb);
    ClosestPair linear = triangle_triangle_closest(pa.v, pb.v);
    Vec2 za = a.project_to_domain(linear.a);
    Vec2 zb = b.project_to_domain(linear.b);
    za = a.clamp_xi(za);
    zb = b.clamp_xi(zb);
    Objective o = objective_face_face(a, b, za, zb);
    bool converged = false;
    int it = 0;
    for (it = 1; it <= 40; ++it) {
        o = objective_face_face(a, b, za, zb);
        if (o.residual <= 1.0e-10) {
            converged = true;
            break;
        }
        Eigen::Matrix4d Hreg = o.H + 1.0e-10 * Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 4, 1> step = -Hreg.fullPivLu().solve(o.grad);
        if (!step.allFinite() || o.grad.dot(step) > 0.0) {
            step = -o.grad / std::max(o.grad.norm(), EPS);
        }
        if (step.norm() <= 1.0e-11) {
            converged = true;
            break;
        }
        double alpha = 1.0;
        bool accepted = false;
        for (int ls = 0; ls < 16; ++ls) {
            Vec2 na = a.clamp_xi(za + alpha * step.segment<2>(0));
            Vec2 nb = b.clamp_xi(zb + alpha * step.segment<2>(2));
            Objective no = objective_face_face(a, b, na, nb);
            if (no.E <= o.E + 1.0e-14) {
                za = na;
                zb = nb;
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!accepted) break;
    }
    o = objective_face_face(a, b, za, zb);
    Vec3 sep = o.Xb - o.Xa;
    double gap = sep.norm();
    Vec3 n = gap <= EPS ? normalize(a.normal(o.xia) - b.normal(o.xib), normalize(pb.centroid() - pa.centroid(), pa.face_normal())) : sep / gap;
    CurvedResult r;
    r.valid = o.Xa.allFinite() && o.Xb.allFinite() && std::isfinite(gap);
    r.contact = r.valid && gap <= d_hat + pa.error_bound + pb.error_bound;
    r.point_a = o.Xa;
    r.point_b = o.Xb;
    r.normal = n;
    r.gap = gap;
    r.bary_a = a.barycentric(o.xia);
    r.bary_b = b.barycentric(o.xib);
    r.iterations = it;
    r.residual = o.residual;
    (void)converged;
    return r;
}

struct DetectionStats {
    int candidate_pairs = 0;
    int contacts = 0;
};

struct ContactSampleCpp {
    Vec3 point_a = Vec3::Zero();
    Vec3 point_b = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
    double gap = 0.0;
    double area_weight = 1.0;
    double response_weight = 1.0;
};

struct SdfSampleCpp {
    double phi = 0.0;
    Vec3 grad = Vec3::UnitZ();
    bool valid = false;
};

struct Se3SdfSampleCpp {
    double phi = 0.0;
    Vec6 grad = Vec6::Zero();
    bool valid = false;

    Vec3 translational_gradient() const { return grad.head<3>(); }
    Vec3 normal(const Vec3& fallback) const { return normalize(translational_gradient(), fallback); }
};

class DenseTricubicSdf {
public:
    using Sampler = std::function<double(const Vec3&)>;

    DenseTricubicSdf() = default;
    DenseTricubicSdf(const Vec3& lo, const Vec3& hi, const std::array<int, 3>& dims, const Sampler& sampler) {
        build(lo, hi, dims, sampler);
    }

    void build(const Vec3& lo, const Vec3& hi, const std::array<int, 3>& dims, const Sampler& sampler) {
        lo_ = lo;
        hi_ = hi;
        nx_ = dims[0];
        ny_ = dims[1];
        nz_ = dims[2];
        if (nx_ < 4 || ny_ < 4 || nz_ < 4) throw std::invalid_argument("DenseTricubicSdf requires at least four samples per axis");
        h_ = Vec3((hi_.x() - lo_.x()) / (nx_ - 1), (hi_.y() - lo_.y()) / (ny_ - 1), (hi_.z() - lo_.z()) / (nz_ - 1));
        values_.assign(static_cast<size_t>(nx_) * ny_ * nz_, 0.0);
        for (int k = 0; k < nz_; ++k) {
            for (int j = 0; j < ny_; ++j) {
                for (int i = 0; i < nx_; ++i) {
                    Vec3 p(lo_.x() + i * h_.x(), lo_.y() + j * h_.y(), lo_.z() + k * h_.z());
                    values_[index(i, j, k)] = sampler(p);
                }
            }
        }
    }

    SdfSampleCpp sample(const Vec3& p) const {
        if (values_.empty()) return {};
        Vec3 u((p.x() - lo_.x()) / h_.x(), (p.y() - lo_.y()) / h_.y(), (p.z() - lo_.z()) / h_.z());
        bool valid = (u.x() >= 1.0 && u.x() <= nx_ - 3.0 && u.y() >= 1.0 && u.y() <= ny_ - 3.0 &&
                      u.z() >= 1.0 && u.z() <= nz_ - 3.0);
        u.x() = clamp(u.x(), 1.0, static_cast<double>(nx_ - 3) - 1.0e-9);
        u.y() = clamp(u.y(), 1.0, static_cast<double>(ny_ - 3) - 1.0e-9);
        u.z() = clamp(u.z(), 1.0, static_cast<double>(nz_ - 3) - 1.0e-9);
        int ix = static_cast<int>(std::floor(u.x()));
        int iy = static_cast<int>(std::floor(u.y()));
        int iz = static_cast<int>(std::floor(u.z()));
        double tx = u.x() - ix;
        double ty = u.y() - iy;
        double tz = u.z() - iz;

        double vx[4][4];
        double dx[4][4];
        for (int kk = 0; kk < 4; ++kk) {
            for (int jj = 0; jj < 4; ++jj) {
                CubicEval cx = cubic(value(ix - 1, iy + jj - 1, iz + kk - 1),
                                     value(ix, iy + jj - 1, iz + kk - 1),
                                     value(ix + 1, iy + jj - 1, iz + kk - 1),
                                     value(ix + 2, iy + jj - 1, iz + kk - 1),
                                     tx);
                vx[kk][jj] = cx.value;
                dx[kk][jj] = cx.derivative;
            }
        }

        double vy[4];
        double dy[4];
        double dxy[4];
        for (int kk = 0; kk < 4; ++kk) {
            CubicEval cy = cubic(vx[kk][0], vx[kk][1], vx[kk][2], vx[kk][3], ty);
            CubicEval cdx = cubic(dx[kk][0], dx[kk][1], dx[kk][2], dx[kk][3], ty);
            vy[kk] = cy.value;
            dy[kk] = cy.derivative;
            dxy[kk] = cdx.value;
        }

        CubicEval cz = cubic(vy[0], vy[1], vy[2], vy[3], tz);
        CubicEval cdy = cubic(dy[0], dy[1], dy[2], dy[3], tz);
        CubicEval cdxz = cubic(dxy[0], dxy[1], dxy[2], dxy[3], tz);
        SdfSampleCpp out;
        out.phi = cz.value;
        out.grad = Vec3(cdxz.value / h_.x(), cdy.value / h_.y(), cz.derivative / h_.z());
        out.valid = valid && std::isfinite(out.phi) && out.grad.allFinite();
        return out;
    }

private:
    struct CubicEval {
        double value = 0.0;
        double derivative = 0.0;
    };

    static CubicEval cubic(double p0, double p1, double p2, double p3, double t) {
        double a0 = 2.0 * p1;
        double a1 = -p0 + p2;
        double a2 = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
        double a3 = -p0 + 3.0 * p1 - 3.0 * p2 + p3;
        CubicEval out;
        out.value = 0.5 * (a0 + a1 * t + a2 * t * t + a3 * t * t * t);
        out.derivative = 0.5 * (a1 + 2.0 * a2 * t + 3.0 * a3 * t * t);
        return out;
    }

    size_t index(int i, int j, int k) const { return static_cast<size_t>(i + nx_ * (j + ny_ * k)); }
    double value(int i, int j, int k) const { return values_[index(i, j, k)]; }

    Vec3 lo_ = Vec3::Zero();
    Vec3 hi_ = Vec3::Zero();
    Vec3 h_ = Vec3::Ones();
    int nx_ = 0;
    int ny_ = 0;
    int nz_ = 0;
    std::vector<double> values_;
};

class DenseSe3TensorCubicSdf {
public:
    using Sampler = std::function<double(const RigidConfiguration&)>;

    DenseSe3TensorCubicSdf() = default;
    DenseSe3TensorCubicSdf(const RigidConfiguration& origin,
                           const Vec6& lo,
                           const Vec6& hi,
                           const std::array<int, 6>& dims,
                           const Sampler& sampler) {
        build(origin, lo, hi, dims, sampler);
    }

    void build(const RigidConfiguration& origin,
               const Vec6& lo,
               const Vec6& hi,
               const std::array<int, 6>& dims,
               const Sampler& sampler) {
        origin_ = origin;
        lo_ = lo;
        hi_ = hi;
        dims_ = dims;
        size_t count = 1;
        for (int d = 0; d < 6; ++d) {
            if (dims_[d] < 5) throw std::invalid_argument("DenseSe3TensorCubicSdf requires at least five samples per axis");
            h_[d] = (hi_[d] - lo_[d]) / (dims_[d] - 1);
            if (std::abs(h_[d]) <= EPS) throw std::invalid_argument("DenseSe3TensorCubicSdf axis has zero extent");
            count *= static_cast<size_t>(dims_[d]);
        }
        values_.assign(count, 0.0);
        std::array<int, 6> id{};
        fill_axis(0, id, sampler);
    }

    Se3SdfSampleCpp sample(const RigidConfiguration& config) const {
        return sample(local_coordinates(config));
    }

    Se3SdfSampleCpp sample(const Vec6& q) const {
        Se3SdfSampleCpp out;
        if (values_.empty()) return out;
        bool valid = true;
        std::array<int, 6> base{};
        std::array<double, 6> t{};
        for (int d = 0; d < 6; ++d) {
            double u = (q[d] - lo_[d]) / h_[d];
            valid = valid && (u >= 1.0 && u <= static_cast<double>(dims_[d]) - 3.0);
            u = clamp(u, 1.0, static_cast<double>(dims_[d] - 3) - 1.0e-9);
            base[d] = static_cast<int>(std::floor(u));
            t[d] = u - base[d];
        }
        out.phi = interpolate_axis(0, base, t, -1);
        for (int d = 0; d < 6; ++d) {
            out.grad[d] = interpolate_axis(0, base, t, d) / h_[d];
        }
        out.valid = valid && std::isfinite(out.phi) && out.grad.allFinite();
        return out;
    }

    RigidConfiguration pose_at(const Vec6& q) const {
        RigidConfiguration out;
        out.position = origin_.position + q.head<3>();
        out.rotation = rotation_from_rotvec(q.tail<3>()) * origin_.rotation;
        out.rotation = orthonormalize(out.rotation);
        return out;
    }

private:
    struct CubicEval {
        double value = 0.0;
        double derivative = 0.0;
    };

    static CubicEval cubic(double p0, double p1, double p2, double p3, double t) {
        double a0 = 2.0 * p1;
        double a1 = -p0 + p2;
        double a2 = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3;
        double a3 = -p0 + 3.0 * p1 - 3.0 * p2 + p3;
        CubicEval out;
        out.value = 0.5 * (a0 + a1 * t + a2 * t * t + a3 * t * t * t);
        out.derivative = 0.5 * (a1 + 2.0 * a2 * t + 3.0 * a3 * t * t);
        return out;
    }

    Vec6 local_coordinates(const RigidConfiguration& config) const {
        Vec6 q = Vec6::Zero();
        q.head<3>() = config.position - origin_.position;
        Mat3 rel = config.rotation * origin_.rotation.transpose();
        Eigen::AngleAxisd aa(rel);
        q.tail<3>() = aa.angle() * aa.axis();
        if (!q.tail<3>().allFinite()) q.tail<3>().setZero();
        return q;
    }

    void fill_axis(int d, std::array<int, 6>& id, const Sampler& sampler) {
        if (d == 6) {
            Vec6 q = Vec6::Zero();
            for (int k = 0; k < 6; ++k) q[k] = lo_[k] + id[k] * h_[k];
            values_[index(id)] = sampler(pose_at(q));
            return;
        }
        for (int i = 0; i < dims_[d]; ++i) {
            id[d] = i;
            fill_axis(d + 1, id, sampler);
        }
    }

    double interpolate_axis(int d,
                            const std::array<int, 6>& base,
                            const std::array<double, 6>& t,
                            int derivative_axis) const {
        if (d == 6) {
            std::array<int, 6> id = current_offsets_;
            for (int k = 0; k < 6; ++k) id[k] += base[k] - 1;
            return values_[index(id)];
        }
        double p[4];
        for (int o = 0; o < 4; ++o) {
            current_offsets_[d] = o;
            p[o] = interpolate_axis(d + 1, base, t, derivative_axis);
        }
        CubicEval c = cubic(p[0], p[1], p[2], p[3], t[d]);
        return d == derivative_axis ? c.derivative : c.value;
    }

    size_t index(const std::array<int, 6>& id) const {
        size_t off = static_cast<size_t>(id[0]);
        size_t stride = static_cast<size_t>(dims_[0]);
        for (int d = 1; d < 6; ++d) {
            off += stride * static_cast<size_t>(id[d]);
            stride *= static_cast<size_t>(dims_[d]);
        }
        return off;
    }

    RigidConfiguration origin_;
    Vec6 lo_ = Vec6::Zero();
    Vec6 hi_ = Vec6::Zero();
    Vec6 h_ = Vec6::Ones();
    std::array<int, 6> dims_{};
    mutable std::array<int, 6> current_offsets_{};
    std::vector<double> values_;
};

class OrientationInvariantSe3TricubicSdf {
public:
    using Sampler = DenseTricubicSdf::Sampler;

    OrientationInvariantSe3TricubicSdf() = default;
    OrientationInvariantSe3TricubicSdf(const Vec3& lo, const Vec3& hi, const std::array<int, 3>& dims, const Sampler& sampler)
        : field_(lo, hi, dims, sampler) {}

    Se3SdfSampleCpp sample(const RigidConfiguration& config) const {
        SdfSampleCpp s = field_.sample(config.position);
        Se3SdfSampleCpp out;
        out.phi = s.phi;
        out.grad.head<3>() = s.grad;
        out.grad.tail<3>().setZero();
        out.valid = s.valid;
        return out;
    }

    SdfSampleCpp sample_translation(const Vec3& center) const {
        SdfSampleCpp out3 = field_.sample(center);
        return out3;
    }

private:
    DenseTricubicSdf field_;
};

static std::pair<std::vector<ContactSampleCpp>, DetectionStats> detect_curved(const Mesh& ma, const Mesh& mb, double d_hat) {
    auto pa = build_primitives(ma, d_hat);
    auto pb = build_primitives(mb, d_hat);
    std::vector<ContactSampleCpp> contacts;
    DetectionStats stats;
    for (const auto& a : pa) {
        for (const auto& b : pb) {
            if (!a.aabb.intersects(b.aabb)) continue;
            stats.candidate_pairs++;
            ClosestPair linear = triangle_triangle_closest(a.v, b.v);
            double eps_pair = a.error_bound + b.error_bound;
            if (linear.distance > d_hat + eps_pair) continue;
            CurvedResult cr = solve_curved_pair(a, b, d_hat);
            if (cr.valid && cr.contact) contacts.push_back({cr.point_a, cr.point_b, cr.normal, cr.gap});
        }
    }
    stats.contacts = static_cast<int>(contacts.size());
    return {contacts, stats};
}

struct RigidBody {
    Vec3 position = Vec3::Zero();
    Mat3 rotation = Mat3::Identity();
    Vec3 linear_velocity = Vec3::Zero();
    Vec3 angular_velocity = Vec3::Zero();
    double mass = 1.0;
    double inertia = 1.0;

    Vec3 point_velocity(const Vec3& p) const { return linear_velocity + angular_velocity.cross(p - position); }
    void integrate(const Vec3& force, const Vec3& torque, double dt) {
        linear_velocity += dt * force / mass;
        position += dt * linear_velocity;
        angular_velocity += dt * torque / inertia;
        rotation = rotation_from_rotvec(angular_velocity * dt) * rotation;
        rotation = orthonormalize(rotation);
    }
};

struct FullRigidBody {
    Vec3 position = Vec3::Zero();
    Mat3 rotation = Mat3::Identity();
    Vec3 linear_velocity = Vec3::Zero();
    Vec3 angular_velocity = Vec3::Zero();
    double mass = 1.0;
    Mat3 inertia_body = Mat3::Identity();
    Mat3 inertia_body_inv = Mat3::Identity();

    Vec3 point_velocity(const Vec3& p) const { return linear_velocity + angular_velocity.cross(p - position); }
    Mat3 world_inertia() const { return rotation * inertia_body * rotation.transpose(); }
    Mat3 world_inertia_inv() const { return rotation * inertia_body_inv * rotation.transpose(); }
    Vec3 angular_acceleration(const Vec3& torque) const {
        Mat3 iw = world_inertia();
        Vec3 angular_momentum = iw * angular_velocity;
        return world_inertia_inv() * (torque - angular_velocity.cross(angular_momentum));
    }
    void integrate(const Vec3& force, const Vec3& torque, double dt) {
        linear_velocity += dt * force / mass;
        position += dt * linear_velocity;
        angular_velocity += dt * angular_acceleration(torque);
        rotation = rotation_from_rotvec(angular_velocity * dt) * rotation;
        rotation = orthonormalize(rotation);
    }
};

struct ContactParams {
    double d_hat = 0.003;
    double kn = 1000.0;
    double cn = 10.0;
    double mu = 0.4;
    double kt = 100.0;
    double ct = 0.0;
    bool penetration_only = false;
    double release_tol = 0.0;
    bool constraint_normal = false;
    double normal_erp = 0.25;
    double normal_solve_time = 0.0;
    double target_gap = 0.0;
    double force_scale = 1.0;
};

struct FrictionState {
    Vec3 xi = Vec3::Zero();
    bool active = false;
};

struct ContactEval {
    Vec3 total_force = Vec3::Zero();
    Vec3 torque = Vec3::Zero();
};

struct SurfaceFrictionState {
    std::vector<FrictionState> samples;

    FrictionState& at(size_t i, size_t n) {
        if (samples.size() != n) samples.assign(n, FrictionState{});
        return samples[i];
    }
};

static ContactEval evaluate_contact(const RigidBody& body,
                                    const Vec3& point,
                                    Vec3 normal,
                                    double gap,
                                    double dt,
                                    const ContactParams& p,
                                    FrictionState& s,
                                    const Vec3& terrain_v = Vec3::Zero(),
                                    const Vec3& external_acc = Vec3::Zero()) {
    normal = normalize(normal);
    const double solve_gap = gap - p.target_gap;
    Vec3 rel_v = body.point_velocity(point) - terrain_v;
    double vn = rel_v.dot(normal);
    const double release_tol = std::max(0.0, p.release_tol);
    const double vn_free = vn + dt * external_acc.dot(normal);
    const bool would_hit = p.constraint_normal && dt > 0.0 && (solve_gap + dt * vn_free <= 0.0);
    bool active = p.penetration_only
                      ? (s.active ? solve_gap <= release_tol : (solve_gap <= 0.0 || would_hit))
                      : (solve_gap <= p.d_hat);
    if (!active) {
        s.active = false;
        if (p.penetration_only && solve_gap > release_tol) {
            s.xi = Vec3::Zero();
        } else {
            s.xi -= s.xi.dot(normal) * normal;
        }
    } else {
        s.active = true;
    }

    double activation = p.penetration_only ? std::max(0.0, -solve_gap) : std::max(0.0, p.d_hat - solve_gap);
    bool damping_active = active && (!p.penetration_only || solve_gap <= 0.0);
    double lambda_penalty = p.kn * activation + (damping_active ? p.cn * std::max(0.0, -vn) : 0.0);
    double lambda_constraint = 0.0;
    if (active && p.constraint_normal && dt > 0.0) {
        Vec3 r = point - body.position;
        double inv_eff_mass = 1.0 / std::max(body.mass, 1.0e-14);
        inv_eff_mass += r.cross(normal).squaredNorm() / std::max(body.inertia, 1.0e-14);
        double vn_min = solve_gap < 0.0 ? p.normal_erp * (-solve_gap) / dt : -solve_gap / dt;
        double impulse = std::max(0.0, (vn_min - vn_free) / std::max(inv_eff_mass, 1.0e-14));
        double solve_time = std::max(dt, p.normal_solve_time);
        lambda_constraint = impulse / solve_time;
        lambda_constraint *= p.force_scale;
    }
    double lambda = std::max(lambda_penalty, lambda_constraint);
    lambda = std::max(0.0, lambda);
    Vec3 normal_force = lambda * normal;
    Vec3 friction_force = Vec3::Zero();
    if (lambda > 0.0 && p.mu > 0.0 && dt > 0.0) {
        Vec3 vt = rel_v - vn * normal;
        Vec3 xi = s.xi + dt * vt;
        xi -= xi.dot(normal) * normal;
        Vec3 trial = -p.kt * xi - p.ct * vt;
        double trial_norm = trial.norm();
        double limit = p.mu * lambda;
        if (trial_norm <= limit || trial_norm <= 1.0e-10) {
            friction_force = trial;
            s.xi = xi;
        } else {
            friction_force = limit * trial / trial_norm;
            s.xi = -friction_force / std::max(p.kt, 1.0e-14);
        }
    } else {
        if (p.penetration_only && solve_gap > release_tol) {
            s.xi = Vec3::Zero();
        } else {
            s.xi -= s.xi.dot(normal) * normal;
        }
    }
    ContactEval out;
    out.total_force = normal_force + friction_force;
    out.torque = (point - body.position).cross(out.total_force);
    return out;
}

static ContactParams scaled_contact_params(ContactParams p, double weight) {
    const double w = std::max(0.0, weight);
    p.kn *= w;
    p.cn *= w;
    p.kt *= w;
    p.ct *= w;
    p.force_scale *= w;
    return p;
}

static ContactEval evaluate_contact_full(const FullRigidBody& body,
                                         const Vec3& point,
                                         Vec3 normal,
                                         double gap,
                                         double dt,
                                         const ContactParams& p,
                                         FrictionState& s,
                                         const Vec3& terrain_v = Vec3::Zero()) {
    normal = normalize(normal);
    Vec3 rel_v = body.point_velocity(point) - terrain_v;
    double vn = rel_v.dot(normal);

    const double activation =
        p.penetration_only ? std::max(0.0, -gap) : std::max(0.0, p.d_hat - gap);
    const bool damping_active = p.penetration_only ? gap <= 0.0 : activation > 0.0;
    double lambda = p.kn * activation + (damping_active ? p.cn * std::max(0.0, -vn) : 0.0);
    lambda = std::max(0.0, lambda);

    Vec3 normal_force = lambda * normal;
    Vec3 friction_force = Vec3::Zero();
    if (lambda > 0.0 && p.mu > 0.0 && dt > 0.0) {
        Vec3 vt = rel_v - vn * normal;
        Vec3 xi = s.xi + dt * vt;
        xi -= xi.dot(normal) * normal;
        Vec3 trial = -p.kt * xi - p.ct * vt;
        double trial_norm = trial.norm();
        double limit = p.mu * lambda;
        if (trial_norm <= limit || trial_norm <= 1.0e-10) {
            friction_force = trial;
            s.xi = xi;
        } else {
            friction_force = limit * trial / trial_norm;
            s.xi = -friction_force / std::max(p.kt, 1.0e-14);
        }
    } else {
        const double release_tol = std::max(0.0, p.release_tol);
        if (p.penetration_only && gap > release_tol) {
            s.xi = Vec3::Zero();
        } else {
            s.xi -= s.xi.dot(normal) * normal;
        }
    }

    ContactEval out;
    out.total_force = normal_force + friction_force;
    out.torque = (point - body.position).cross(out.total_force);
    return out;
}

static std::tuple<Vec3, Vec3, Vec3> basis_from_axis(const Vec3& axis_in) {
    Vec3 w = normalize(axis_in, Vec3(0.0, 0.0, -1.0));
    Vec3 ref = std::abs(w.x()) < 0.85 ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
    Vec3 u = normalize(ref.cross(w), Vec3(0.0, 1.0, 0.0));
    Vec3 v = normalize(w.cross(u), Vec3(1.0, 0.0, 0.0));
    return {u, v, w};
}

static Mesh sphere_patch(const Vec3& center, double radius, const Vec3& contact_normal, double patch_radius, int n, const std::string& name) {
    auto [u, v, axis] = basis_from_axis(-normalize(contact_normal));
    Mesh m;
    m.name = name;
    for (int j = 0; j <= n; ++j) {
        double b = -patch_radius + 2.0 * patch_radius * j / n;
        for (int i = 0; i <= n; ++i) {
            double a = -patch_radius + 2.0 * patch_radius * i / n;
            Vec3 radial = normalize(axis + (a / radius) * u + (b / radius) * v, axis);
            m.normals.push_back(radial);
            m.vertices.push_back(center + radius * radial);
        }
    }
    auto vid = [&](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            m.faces.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
            m.faces.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
        }
    }
    return m;
}

static Mesh box_mesh(double hx, double hy, double hz, const std::string& name) {
    Mesh m;
    m.name = name;
    auto add_quad = [&](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& normal) {
        int base = static_cast<int>(m.vertices.size());
        m.vertices.push_back(a);
        m.vertices.push_back(b);
        m.vertices.push_back(c);
        m.vertices.push_back(d);
        for (int i = 0; i < 4; ++i) m.normals.push_back(normal);
        m.faces.push_back({base, base + 1, base + 2});
        m.faces.push_back({base, base + 2, base + 3});
    };
    add_quad(Vec3(-hx, -hy, hz), Vec3(hx, -hy, hz), Vec3(hx, hy, hz), Vec3(-hx, hy, hz), Vec3(0.0, 0.0, 1.0));
    add_quad(Vec3(-hx, hy, -hz), Vec3(hx, hy, -hz), Vec3(hx, -hy, -hz), Vec3(-hx, -hy, -hz), Vec3(0.0, 0.0, -1.0));
    add_quad(Vec3(-hx, -hy, -hz), Vec3(hx, -hy, -hz), Vec3(hx, -hy, hz), Vec3(-hx, -hy, hz), Vec3(0.0, -1.0, 0.0));
    add_quad(Vec3(hx, -hy, -hz), Vec3(hx, hy, -hz), Vec3(hx, hy, hz), Vec3(hx, -hy, hz), Vec3(1.0, 0.0, 0.0));
    add_quad(Vec3(hx, hy, -hz), Vec3(-hx, hy, -hz), Vec3(-hx, hy, hz), Vec3(hx, hy, hz), Vec3(0.0, 1.0, 0.0));
    add_quad(Vec3(-hx, hy, -hz), Vec3(-hx, -hy, -hz), Vec3(-hx, -hy, hz), Vec3(-hx, hy, hz), Vec3(-1.0, 0.0, 0.0));
    return m;
}

static ContactSampleCpp select_contact(const std::vector<ContactSampleCpp>& contacts, const Vec3& body_pos) {
    return *std::min_element(contacts.begin(), contacts.end(), [&](const auto& a, const auto& b) {
        double oa = normalize(a.normal).dot(body_pos - a.point_a);
        double ob = normalize(b.normal).dot(body_pos - b.point_a);
        if (a.gap != b.gap) return a.gap < b.gap;
        return -oa < -ob;
    });
}

struct BenchResult {
    std::string name;
    double elapsed_ms = 0.0;
    int steps = 0;
    int contacts = 0;
    int pairs = 0;
    Vec3 final_position = Vec3::Zero();
    Vec3 final_velocity = Vec3::Zero();
    double final_gap = 0.0;
};

struct TraceWriter {
    std::ofstream file;

    TraceWriter(const std::string& output_dir, const std::string& name) {
        if (output_dir.empty()) return;
        std::filesystem::create_directories(output_dir);
        file.open((std::filesystem::path(output_dir) / (name + ".csv")).string());
        file << std::setprecision(17);
        file << "source,step,time,x,y,z,vx,vy,vz,ax,ay,az,wx,wy,wz,gap,normal_x,normal_y,normal_z,"
                "contact_force_x,contact_force_y,contact_force_z,contact_force,contacts,candidate_pairs\n";
    }

    void write(const std::string& source,
               int step,
               double time,
               const RigidBody& body,
               const Vec3& acceleration,
               const ContactSampleCpp& sample,
               const Vec3& force,
               int contacts,
               int pairs) {
        if (!file) return;
        file << source << ',' << step << ',' << time << ','
             << body.position.x() << ',' << body.position.y() << ',' << body.position.z() << ','
             << body.linear_velocity.x() << ',' << body.linear_velocity.y() << ',' << body.linear_velocity.z() << ','
             << acceleration.x() << ',' << acceleration.y() << ',' << acceleration.z() << ','
             << body.angular_velocity.x() << ',' << body.angular_velocity.y() << ',' << body.angular_velocity.z() << ','
             << sample.gap << ','
             << sample.normal.x() << ',' << sample.normal.y() << ',' << sample.normal.z() << ','
             << force.x() << ',' << force.y() << ',' << force.z() << ',' << force.norm() << ','
             << contacts << ',' << pairs << '\n';
    }
};

class ContactGeometry {
public:
    virtual ~ContactGeometry() = default;
    virtual std::string id() const = 0;
    virtual double sphere_radius() const { return 0.0; }
    virtual double bounding_radius() const { return sphere_radius(); }
    virtual Mesh local_patch(const RigidBody& body, double sphere_radius, const Vec3& normal_hint, double d_hat) const = 0;
    virtual Mesh local_patch(const RigidConfiguration& body_config, double search_radius, const Vec3& normal_hint, double d_hat) const {
        RigidBody sphere_body;
        sphere_body.position = body_config.position;
        sphere_body.rotation = body_config.rotation;
        return local_patch(sphere_body, search_radius, normal_hint, d_hat);
    }
    virtual bool supports_sphere_response() const { return false; }
    virtual bool supports_direct_response(const ContactGeometry& moving_geometry) const {
        return supports_sphere_response() && moving_geometry.sphere_radius() > 0.0;
    }
    virtual ContactSampleCpp direct_response_sample(const RigidConfiguration& body_config,
                                                    const ContactGeometry& moving_geometry,
                                                    const Vec3& normal_hint,
                                                    double d_hat) const {
        (void)normal_hint;
        (void)d_hat;
        double r = moving_geometry.sphere_radius();
        if (supports_sphere_response() && r > 0.0) {
            RigidBody sphere_body;
            sphere_body.position = body_config.position;
            sphere_body.rotation = body_config.rotation;
            return closest_sphere_contact(sphere_body, r);
        }
        throw std::logic_error("direct_response_sample is not defined for this geometry pair");
    }
    virtual ContactSampleCpp closest_sphere_contact(const RigidBody& sphere_body, double sphere_radius) const {
        (void)sphere_body;
        (void)sphere_radius;
        throw std::logic_error("closest_sphere_contact is not defined for this geometry");
    }
    virtual Vec3 surface_velocity(const Vec3& world_point) const {
        (void)world_point;
        return Vec3::Zero();
    }
};

class RigidMeshGeometry final : public ContactGeometry {
public:
    RigidMeshGeometry(std::string id, Mesh body_frame_mesh)
        : id_(std::move(id)), body_frame_mesh_(std::move(body_frame_mesh)), bounding_radius_(mesh_bounding_radius(body_frame_mesh_)) {
        if (body_frame_mesh_.name.empty()) body_frame_mesh_.name = id_;
    }

    std::string id() const override { return id_; }
    double bounding_radius() const override { return bounding_radius_; }
    Mesh local_patch(const RigidBody& body, double search_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)search_radius;
        (void)normal_hint;
        (void)d_hat;
        return transform_mesh(body_frame_mesh_, make_configuration(body.position, body.rotation), id_ + "_world");
    }
    Mesh local_patch(const RigidConfiguration& body_config, double search_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)search_radius;
        (void)normal_hint;
        (void)d_hat;
        return transform_mesh(body_frame_mesh_, body_config, id_ + "_world");
    }

private:
    std::string id_;
    Mesh body_frame_mesh_;
    double bounding_radius_ = 0.0;
};

class ContactResponseField {
public:
    virtual ~ContactResponseField() = default;
    virtual bool sample(const RigidConfiguration& body_config,
                        const ContactSampleCpp& detector_or_fallback,
                        ContactSampleCpp& out) const = 0;
};

class DenseSe3ContactResponseField final : public ContactResponseField {
public:
    using Sampler = DenseSe3TensorCubicSdf::Sampler;

    DenseSe3ContactResponseField(const RigidConfiguration& origin,
                                 const Vec6& lo,
                                 const Vec6& hi,
                                 const std::array<int, 6>& dims,
                                 const Sampler& sampler)
        : sdf_(origin, lo, hi, dims, sampler) {}

    bool sample(const RigidConfiguration& body_config,
                const ContactSampleCpp& detector_or_fallback,
                ContactSampleCpp& out) const override {
        Se3SdfSampleCpp s = sdf_.sample(body_config);
        if (!s.valid || !std::isfinite(s.phi) || !s.grad.allFinite()) return false;
        out = detector_or_fallback;
        Vec3 normal = s.normal(detector_or_fallback.normal);
        if (normal.dot(detector_or_fallback.normal) < 0.0) normal = -normal;
        out.normal = normal;
        out.gap = s.phi;
        return true;
    }

private:
    DenseSe3TensorCubicSdf sdf_;
};

class FunctionalContactResponseField final : public ContactResponseField {
public:
    using Sampler = std::function<Se3SdfSampleCpp(const RigidConfiguration&)>;

    explicit FunctionalContactResponseField(Sampler sampler) : sampler_(std::move(sampler)) {}

    bool sample(const RigidConfiguration& body_config,
                const ContactSampleCpp& detector_or_fallback,
                ContactSampleCpp& out) const override {
        Se3SdfSampleCpp s = sampler_(body_config);
        if (!s.valid || !std::isfinite(s.phi) || !s.grad.allFinite()) return false;
        out = detector_or_fallback;
        Vec3 normal = s.normal(detector_or_fallback.normal);
        if (normal.dot(detector_or_fallback.normal) < 0.0) normal = -normal;
        out.normal = normal;
        out.gap = s.phi;
        return true;
    }

private:
    Sampler sampler_;
};

struct ContactPairSpec {
    std::string id;
    int terrain_geometry = -1;
    int body_geometry = -1;
    double detection_d_hat = 0.0;
    ContactParams params;
    Vec3 normal_hint = Vec3(0.0, 0.0, 1.0);
    std::shared_ptr<const ContactResponseField> response_field;
    bool surface_to_surface = true;
    double surface_patch_radius = 0.0;
    int surface_quadrature_order = 2;
    double surface_outer_fraction = 0.10;
    int detector_stride = 1;
};

struct ContactScene {
    std::string source;
    double dt = 1.0e-3;
    Vec3 gravity = Vec3::Zero();
    RigidBody body;
    std::vector<std::unique_ptr<ContactGeometry>> geometries;
    std::vector<ContactPairSpec> pairs;
    std::function<void(RigidBody&)> post_integrate;
};

static ContactSampleCpp orient_sample(ContactSampleCpp sample, const Vec3& body_pos) {
    sample.normal = normalize(sample.normal);
    if (sample.normal.dot(body_pos - sample.point_a) < 0.0) sample.normal = -sample.normal;
    return sample;
}

static ContactSampleCpp signed_detector_sample(ContactSampleCpp sample, const Vec3& body_pos) {
    sample = orient_sample(sample, body_pos);
    sample.gap = (sample.point_b - sample.point_a).dot(sample.normal);
    return sample;
}

struct ContactQueryResult {
    ContactSampleCpp detector;
    ContactSampleCpp response;
    DetectionStats stats;
    bool has_detector = false;
    bool has_response = false;
};

struct ContactPatchCpp {
    std::vector<ContactSampleCpp> samples;
    ContactSampleCpp representative;
    DetectionStats stats;
    bool has_detector = false;
    bool has_response = false;
    double area = 0.0;
};

static ContactQueryResult query_contact_pair(const ContactGeometry& terrain,
                                             const ContactGeometry& moving,
                                             const RigidConfiguration& body_config,
                                             const ContactPairSpec& pair,
                                             bool run_detector = true,
                                             bool assume_detector_active = false) {
    ContactQueryResult out;
    const double search_radius = std::max(0.0, moving.bounding_radius());
    const bool direct_response = terrain.supports_direct_response(moving);
    if (direct_response) {
        out.response = orient_sample(terrain.direct_response_sample(body_config, moving, pair.normal_hint, pair.detection_d_hat),
                                     body_config.position);
        out.has_response = true;
    }

    if (run_detector) {
        Mesh terrain_mesh = terrain.local_patch(body_config, search_radius, pair.normal_hint, pair.detection_d_hat);
        Mesh body_mesh = moving.local_patch(body_config, search_radius, pair.normal_hint, pair.detection_d_hat);
        auto [contacts, stats] = detect_curved(terrain_mesh, body_mesh, pair.detection_d_hat);
        out.stats = stats;

        if (!contacts.empty()) {
            out.detector = signed_detector_sample(select_contact(contacts, body_config.position), body_config.position);
            out.has_detector = true;
            if (!out.has_response) {
                out.response = out.detector;
                out.has_response = true;
            }
        }
    } else {
        out.has_detector = assume_detector_active;
    }

    if (!out.has_response) {
        out.response.point_a = body_config.position - pair.normal_hint * search_radius;
        out.response.point_b = body_config.position;
        out.response.normal = normalize(pair.normal_hint);
        out.response.gap = std::numeric_limits<double>::infinity();
        out.has_response = true;
    }

    if (!out.has_detector && pair.params.penetration_only && out.has_response) {
        out.response.gap = std::max(out.response.gap, std::max(0.0, pair.params.release_tol) + 1.0e-6);
    }
    if (pair.response_field && out.has_response) {
        ContactSampleCpp field_response;
        if (pair.response_field->sample(body_config, out.response, field_response)) {
            out.response = orient_sample(field_response, body_config.position);
        }
    }
    return out;
}

static double default_surface_patch_radius(const ContactGeometry& moving, const ContactPairSpec& pair) {
    if (pair.surface_patch_radius > 0.0) return pair.surface_patch_radius;
    const double r = moving.sphere_radius();
    if (r > 0.0) return clamp(0.22 * r, 0.008, 0.028);
    return 0.012;
}

static std::vector<std::tuple<Vec2, double>> square_patch_quadrature(int order, double radius, double outer_fraction = 0.10) {
    order = std::max(1, std::min(order, 3));
    outer_fraction = clamp(outer_fraction, 0.0, 1.0);
    std::vector<double> x;
    std::vector<double> w;
    if (order == 1) {
        x = {0.0};
        w = {2.0};
    } else if (order == 2) {
        const double a = 0.62 * radius;
        const double b = 0.36 * radius;
        const double area = 4.0 * radius * radius;
        return {
            {Vec2(0.0, 0.0), (1.0 - outer_fraction) * area},
            {Vec2(0.0, 2.0 * b), (outer_fraction / 3.0) * area},
            {Vec2(-a, -b), (outer_fraction / 3.0) * area},
            {Vec2(a, -b), (outer_fraction / 3.0) * area},
        };
    } else {
        const double a = std::sqrt(3.0 / 5.0);
        x = {-a, 0.0, a};
        w = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};
    }
    std::vector<std::tuple<Vec2, double>> nodes;
    nodes.reserve(x.size() * x.size());
    for (size_t j = 0; j < x.size(); ++j) {
        for (size_t i = 0; i < x.size(); ++i) {
            nodes.emplace_back(Vec2(radius * x[i], radius * x[j]), radius * radius * w[i] * w[j]);
        }
    }
    return nodes;
}

static ContactPatchCpp query_surface_contact_patch(const ContactGeometry& terrain,
                                                   const ContactGeometry& moving,
                                                   const RigidConfiguration& body_config,
                                                   const ContactPairSpec& pair,
                                                   bool run_detector = true,
                                                   bool assume_detector_active = false) {
    ContactQueryResult center = query_contact_pair(terrain, moving, body_config, pair, run_detector, assume_detector_active);
    ContactPatchCpp patch;
    patch.stats = center.stats;
    patch.has_detector = center.has_detector;
    patch.has_response = center.has_response;
    if (!center.has_response) return patch;

    ContactSampleCpp central = orient_sample(center.response, body_config.position);
    if (!pair.surface_to_surface || pair.surface_quadrature_order <= 1) {
        central.area_weight = 1.0;
        central.response_weight = 1.0;
        patch.samples.push_back(central);
        patch.representative = central;
        patch.area = 1.0;
        return patch;
    }

    const double radius = default_surface_patch_radius(moving, pair);
    auto [t1, t2, nc] = basis_from_axis(central.normal);
    (void)nc;
    auto nodes = square_patch_quadrature(pair.surface_quadrature_order, radius, pair.surface_outer_fraction);
    double area_sum = 0.0;
    for (const auto& node : nodes) area_sum += std::get<1>(node);
    if (area_sum <= 0.0) area_sum = 1.0;

    patch.samples.reserve(nodes.size());
    for (const auto& node : nodes) {
        Vec2 uv = std::get<0>(node);
        double area_w = std::get<1>(node);
        Vec3 offset = uv.x() * t1 + uv.y() * t2;
        RigidConfiguration shifted = body_config;
        shifted.position += offset;

        ContactSampleCpp sample = central;
        sample.point_a += offset;
        sample.point_b += offset;
        sample.area_weight = area_w;
        sample.response_weight = area_w / area_sum;

        bool refined = false;
        if (pair.response_field) {
            ContactSampleCpp seed = sample;
            ContactSampleCpp field_sample;
            if (pair.response_field->sample(shifted, seed, field_sample)) {
                sample = orient_sample(field_sample, shifted.position);
                sample.point_a = seed.point_a;
                sample.point_b = seed.point_b;
                sample.area_weight = area_w;
                sample.response_weight = area_w / area_sum;
                refined = true;
            }
        }
        if (!refined && terrain.supports_direct_response(moving)) {
            sample = orient_sample(terrain.direct_response_sample(shifted, moving, central.normal, pair.detection_d_hat),
                                   shifted.position);
            sample.area_weight = area_w;
            sample.response_weight = area_w / area_sum;
        }
        if (!center.has_detector && pair.params.penetration_only) {
            sample.gap = std::max(sample.gap, std::max(0.0, pair.params.release_tol) + 1.0e-6);
        }
        patch.samples.push_back(sample);
    }
    patch.area = area_sum;
    patch.representative = *std::min_element(patch.samples.begin(), patch.samples.end(), [](const auto& a, const auto& b) {
        return a.gap < b.gap;
    });
    return patch;
}

static ContactSampleCpp trace_contact_sample(const ContactGeometry& terrain,
                                             const ContactGeometry& moving,
                                             const RigidConfiguration& body_config,
                                             const ContactPairSpec& pair) {
    if (pair.surface_to_surface && pair.surface_quadrature_order > 1) {
        ContactPatchCpp patch = query_surface_contact_patch(terrain, moving, body_config, pair);
        if (!patch.samples.empty()) return patch.representative;
    }
    if (pair.response_field) {
        return query_contact_pair(terrain, moving, body_config, pair).response;
    }
    if (terrain.supports_direct_response(moving)) {
        return orient_sample(terrain.direct_response_sample(body_config, moving, pair.normal_hint, pair.detection_d_hat),
                             body_config.position);
    }
    return query_contact_pair(terrain, moving, body_config, pair).response;
}

static BenchResult run_contact_scene(ContactScene& scene, int steps, const std::string& output_dir = "") {
    RigidBody body = scene.body;
    std::map<std::string, SurfaceFrictionState> states;
    std::map<std::string, bool> detector_active_cache;
    TraceWriter trace(output_dir, scene.source);
    int total_contacts = 0;
    int total_pairs = 0;
    double last_gap = 0.0;
    auto t0 = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) {
        Vec3 contact_force = Vec3::Zero();
        Vec3 total_torque = Vec3::Zero();
        ContactSampleCpp best;
        best.gap = std::numeric_limits<double>::infinity();
        int step_contacts = 0;
        int step_pairs = 0;

        for (ContactPairSpec& pair : scene.pairs) {
            const ContactGeometry& terrain = *scene.geometries.at(static_cast<size_t>(pair.terrain_geometry));
            const ContactGeometry& body_geometry = *scene.geometries.at(static_cast<size_t>(pair.body_geometry));
            const int stride = std::max(1, pair.detector_stride);
            const bool cached_active = detector_active_cache[pair.id];
            const bool run_detector = (step % stride == 0) || !cached_active;
            ContactPatchCpp patch =
                query_surface_contact_patch(terrain,
                                            body_geometry,
                                            make_configuration(body.position, body.rotation),
                                            pair,
                                            run_detector,
                                            cached_active);
            if (run_detector) detector_active_cache[pair.id] = patch.has_detector;
            total_contacts += patch.stats.contacts;
            total_pairs += patch.stats.candidate_pairs;
            step_contacts += patch.stats.contacts;
            step_pairs += patch.stats.candidate_pairs;

            SurfaceFrictionState& pair_state = states[pair.id];
            for (size_t qi = 0; qi < patch.samples.size(); ++qi) {
                ContactSampleCpp sample = patch.samples[qi];
                Vec3 terrain_velocity = terrain.surface_velocity(sample.point_a);
                ContactParams weighted_params = scaled_contact_params(pair.params, sample.response_weight);
                ContactEval c = evaluate_contact(body,
                                                 sample.point_b,
                                                 sample.normal,
                                                 sample.gap,
                                                 scene.dt,
                                                 weighted_params,
                                                 pair_state.at(qi, patch.samples.size()),
                                                 terrain_velocity,
                                                 scene.gravity);
                contact_force += c.total_force;
                total_torque += c.torque;
                if (sample.gap < best.gap) best = sample;
            }
            if (!patch.samples.empty()) {
                pair.normal_hint = patch.representative.normal;
            }
        }

        Vec3 previous_linear_velocity = body.linear_velocity;
        Vec3 total_force = body.mass * scene.gravity + contact_force;
        body.integrate(total_force, total_torque, scene.dt);
        if (scene.post_integrate) scene.post_integrate(body);
        Vec3 realized_acceleration = (body.linear_velocity - previous_linear_velocity) / scene.dt;
        ContactSampleCpp trace_sample;
        trace_sample.gap = std::numeric_limits<double>::infinity();
        for (const ContactPairSpec& pair : scene.pairs) {
            const ContactGeometry& terrain = *scene.geometries.at(static_cast<size_t>(pair.terrain_geometry));
            const ContactGeometry& body_geometry = *scene.geometries.at(static_cast<size_t>(pair.body_geometry));
            ContactSampleCpp sample =
                trace_contact_sample(terrain, body_geometry, make_configuration(body.position, body.rotation), pair);
            if (sample.gap < trace_sample.gap) trace_sample = sample;
        }
        if (!std::isfinite(trace_sample.gap)) trace_sample = best;
        last_gap = trace_sample.gap;
        trace.write(scene.source, step, (step + 1) * scene.dt, body, realized_acceleration, trace_sample, contact_force, step_contacts, step_pairs);
    }
    auto t1 = std::chrono::steady_clock::now();
    return {
        scene.source,
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        steps,
        total_contacts,
        total_pairs,
        body.position,
        body.linear_velocity,
        last_gap,
    };
}

namespace Guide {
constexpr double AMP = 0.10;
constexpr double WAVELENGTH = 1.45;
constexpr double GROOVE_K = 1.55;
constexpr double X0 = -1.0, X1 = 1.0, Y0 = -0.42, Y1 = 0.42;
static double z(double x, double y) { return AMP * std::sin(2.0 * PI * x / WAVELENGTH) + GROOVE_K * y * y; }
static double dzdx(double x) { return AMP * (2.0 * PI / WAVELENGTH) * std::cos(2.0 * PI * x / WAVELENGTH); }
static double dzdy(double y) { return 2.0 * GROOVE_K * y; }
static double d2zdx2(double x) {
    const double k = 2.0 * PI / WAVELENGTH;
    return -AMP * k * k * std::sin(k * x);
}
static double d2zdy2() { return 2.0 * GROOVE_K; }
static Vec3 normal(double x, double y) {
    return normalize(Vec3(-dzdx(x), -dzdy(y), 1.0));
}
static Vec3 point(double x, double y) { return Vec3(x, y, z(x, y)); }

struct ClosestPoint {
    double u = 0.0;
    double v = 0.0;
    Vec3 point = Vec3::Zero();
    Vec3 normal = Vec3(0.0, 0.0, 1.0);
    double distance = 0.0;
};

static ClosestPoint closest_point(const Vec3& pos) {
    double u = clamp(pos.x(), X0, X1);
    double v = clamp(pos.y(), Y0, Y1);
    for (int iter = 0; iter < 14; ++iter) {
        Vec3 q = point(u, v);
        Vec3 r = q - pos;
        const double hx = dzdx(u);
        const double hy = dzdy(v);
        Vec3 qu(1.0, 0.0, hx);
        Vec3 qv(0.0, 1.0, hy);
        const double f0 = r.dot(qu);
        const double f1 = r.dot(qv);
        const double j00 = qu.dot(qu) + r.z() * d2zdx2(u);
        const double j01 = qu.dot(qv);
        const double j10 = j01;
        const double j11 = qv.dot(qv) + r.z() * d2zdy2();
        const double det = j00 * j11 - j01 * j10;
        if (std::abs(det) < 1.0e-14) break;
        double du = (-f0 * j11 + j01 * f1) / det;
        double dv = (j10 * f0 - j00 * f1) / det;
        const double step_norm = std::sqrt(du * du + dv * dv);
        const double max_step = 0.15;
        if (step_norm > max_step) {
            const double scale = max_step / step_norm;
            du *= scale;
            dv *= scale;
        }
        u = clamp(u + du, X0, X1);
        v = clamp(v + dv, Y0, Y1);
        if (std::sqrt(du * du + dv * dv) < 1.0e-12) break;
    }
    ClosestPoint cp;
    cp.u = u;
    cp.v = v;
    cp.point = point(u, v);
    cp.distance = (pos - cp.point).norm();
    cp.normal = cp.distance > 1.0e-14 ? (pos - cp.point) / cp.distance : normal(u, v);
    Vec3 oriented = normal(u, v);
    if (cp.normal.dot(oriented) < 0.0) cp.normal = -cp.normal;
    return cp;
}

static std::tuple<double, Vec3, Vec3> gap_normal_point(const Vec3& pos, double r) {
    ClosestPoint cp = closest_point(pos);
    return {cp.distance - r, cp.normal, cp.point};
}

static std::pair<double, Vec3> gap_normal(const Vec3& pos, double r) {
    auto [gap, n, q] = gap_normal_point(pos, r);
    (void)q;
    return {gap, n};
}

static const OrientationInvariantSe3TricubicSdf& configuration_sdf() {
    static const OrientationInvariantSe3TricubicSdf sdf(
        Vec3(-1.08, -0.50, -0.30),
        Vec3(1.08, 0.50, 0.56),
        std::array<int, 3>{201, 101, 121},
        [](const Vec3& p) { return std::get<0>(gap_normal_point(p, 0.09)); });
    return sdf;
}

static ContactSampleCpp sdf_sphere_contact(const Vec3& center, double radius) {
    auto [fallback_gap, fallback_normal, fallback_point] = gap_normal_point(center, radius);
    Se3SdfSampleCpp s = configuration_sdf().sample(make_configuration(center));
    if (!s.valid || !std::isfinite(s.phi) || !s.grad.allFinite()) {
        return ContactSampleCpp{fallback_point, center - radius * fallback_normal, fallback_normal, fallback_gap};
    }
    Vec3 normal = s.normal(fallback_normal);
    if (normal.dot(fallback_normal) < 0.0) normal = -normal;
    double gap = s.phi;
    Vec3 surface_point = center - (radius + gap) * normal;
    return ContactSampleCpp{surface_point, center - radius * normal, normal, gap};
}

struct Cache {
    int nx = 18, ny = 8;
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    int vid(int i, int j) const { return j * (nx + 1) + i; }
    static Cache build(int nx, int ny) {
        Cache c;
        c.nx = nx;
        c.ny = ny;
        for (int j = 0; j <= ny; ++j) {
            double y = Y0 + (Y1 - Y0) * j / ny;
            for (int i = 0; i <= nx; ++i) {
                double x = X0 + (X1 - X0) * i / nx;
                c.vertices.push_back(point(x, y));
                c.normals.push_back(normal(x, y));
            }
        }
        return c;
    }
    Mesh local_mesh(const Vec3& center, double margin) const {
        double hx = (X1 - X0) / nx;
        double hy = (Y1 - Y0) / ny;
        int i0 = std::max(0, static_cast<int>(std::floor((center.x() - margin - X0) / hx)));
        int i1 = std::min(nx - 1, static_cast<int>(std::floor((center.x() + margin - X0) / hx)));
        int j0 = std::max(0, static_cast<int>(std::floor((center.y() - margin - Y0) / hy)));
        int j1 = std::min(ny - 1, static_cast<int>(std::floor((center.y() + margin - Y0) / hy)));
        Mesh m;
        m.name = "guide_track_local";
        std::unordered_map<int, int> used;
        auto add = [&](int gid) {
            auto it = used.find(gid);
            if (it != used.end()) return it->second;
            int id = static_cast<int>(m.vertices.size());
            used[gid] = id;
            m.vertices.push_back(vertices[gid]);
            m.normals.push_back(normals[gid]);
            return id;
        };
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                int v00 = add(vid(i, j));
                int v10 = add(vid(i + 1, j));
                int v01 = add(vid(i, j + 1));
                int v11 = add(vid(i + 1, j + 1));
                m.faces.push_back({v00, v10, v11});
                m.faces.push_back({v00, v11, v01});
            }
        }
        return m;
    }
};
}  // namespace Guide

class SphereGeometry final : public ContactGeometry {
public:
    SphereGeometry(std::string id, double radius, double patch_radius, int patch_resolution)
        : id_(std::move(id)), radius_(radius), patch_radius_(patch_radius), patch_resolution_(patch_resolution) {}

    std::string id() const override { return id_; }
    double sphere_radius() const override { return radius_; }
    Mesh local_patch(const RigidBody& body, double sphere_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)sphere_radius;
        (void)d_hat;
        return sphere_patch(body.position, radius_, normal_hint, patch_radius_, patch_resolution_, id_);
    }

private:
    std::string id_;
    double radius_ = 0.0;
    double patch_radius_ = 0.0;
    int patch_resolution_ = 1;
};

class GuideHeightFieldGeometry final : public ContactGeometry {
public:
    GuideHeightFieldGeometry() : cache_(Guide::Cache::build(18, 8)) {}

    std::string id() const override { return "guide_track"; }
    bool supports_sphere_response() const override { return true; }
    Mesh local_patch(const RigidBody& body, double sphere_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)normal_hint;
        return cache_.local_mesh(body.position, sphere_radius + d_hat);
    }
    ContactSampleCpp closest_sphere_contact(const RigidBody& body, double sphere_radius) const override {
        return Guide::sdf_sphere_contact(body.position, sphere_radius);
    }

private:
    Guide::Cache cache_;
};

static ContactScene make_guide_scene() {
    constexpr double radius = 0.09;
    constexpr double mass = 0.75;
    constexpr double initial_gap = 0.002;
    Vec3 p0 = Guide::point(-0.74, 0.02);
    Vec3 n0 = Guide::normal(-0.74, 0.02);

    ContactScene scene;
    scene.source = "guide_slot_cpp";
    scene.dt = 5.0e-4;
    scene.gravity = Vec3(0.0, 0.0, -9.81);
    scene.body.mass = mass;
    scene.body.inertia = 0.4 * mass * radius * radius;
    scene.body.position = p0 + n0 * (radius + initial_gap);
    scene.body.linear_velocity = Vec3(0.58, 0.0, 0.0);
    scene.body.linear_velocity -= scene.body.linear_velocity.dot(n0) * n0;
    scene.body.angular_velocity = n0.cross(scene.body.linear_velocity) / radius;
    scene.geometries.push_back(std::make_unique<GuideHeightFieldGeometry>());
    scene.geometries.push_back(std::make_unique<SphereGeometry>("sphere_patch", radius, 0.035, 1));
    ContactPairSpec guide_pair{
        "guide_track",
        0,
        1,
        0.018,
        ContactParams{0.002, 8.0e4, 520.0, g_guide_mu, 1200.0, 4.0, true, 5.0e-4, true, 0.25, 1.0e-3, g_guide_target_gap},
        n0,
    };
    guide_pair.response_field =
        std::make_shared<FunctionalContactResponseField>([](const RigidConfiguration& q) { return Guide::configuration_sdf().sample(q); });
    guide_pair.surface_patch_radius = 0.0035;
    guide_pair.surface_quadrature_order = 2;
    guide_pair.surface_outer_fraction = 1.0e-3;
    guide_pair.detector_stride = 5;
    scene.pairs.push_back(guide_pair);
    scene.post_integrate = [](RigidBody& body) {
        constexpr double radius_local = 0.09;
        auto [gap, normal, surface_point] = Guide::gap_normal_point(body.position, radius_local);
        const double lower_gap = std::min(-0.012, g_guide_target_gap - 0.003);
        if (gap < lower_gap) {
            body.position = surface_point + normal * (radius_local + lower_gap);
            double vn = body.linear_velocity.dot(normal);
            if (vn < 0.0) body.linear_velocity -= vn * normal;
        }
    };
    return scene;
}

static BenchResult run_guide(int steps, const std::string& output_dir = "") {
    (void)Guide::configuration_sdf();
    ContactScene scene = make_guide_scene();
    return run_contact_scene(scene, steps, output_dir);
}

static ContactScene make_generic_mesh_smoke_scene() {
    Vec3 p0 = Guide::point(-0.72, 0.01);
    Vec3 n0 = Guide::normal(-0.72, 0.01);
    ContactScene scene;
    scene.source = "generic_mesh_contact_cpp";
    scene.dt = 5.0e-4;
    scene.gravity = Vec3(0.0, 0.0, -9.81);
    scene.body.mass = 0.80;
    scene.body.inertia = 1.0e-3;
    scene.body.position = p0 + n0 * 0.035;
    scene.body.rotation = rotation_from_rotvec(Vec3(0.0, 0.10, 0.05));
    scene.body.linear_velocity = Vec3(0.16, 0.0, 0.0);
    scene.geometries.push_back(std::make_unique<GuideHeightFieldGeometry>());
    scene.geometries.push_back(std::make_unique<RigidMeshGeometry>("box_contact", box_mesh(0.030, 0.024, 0.018, "box_contact_body")));
    scene.pairs.push_back(ContactPairSpec{
        "generic_mesh_guide",
        0,
        1,
        0.020,
        ContactParams{0.006, 3.0e4, 120.0, 0.30, 300.0, 1.0, false},
        n0,
    });
    return scene;
}

static BenchResult run_generic_mesh_smoke(int steps, const std::string& output_dir = "") {
    ContactScene scene = make_generic_mesh_smoke_scene();
    return run_contact_scene(scene, steps, output_dir);
}

namespace Socket {
constexpr double R = 0.42;
static Vec3 direction(const Vec3& p) { return normalize(p, Vec3(0.0, 0.0, -1.0)); }
static Vec3 point(const Vec3& d) { return R * normalize(d, Vec3(0.0, 0.0, -1.0)); }
static std::tuple<double, Vec3, Vec3> gap_normal(const Vec3& pos, double radius) {
    Vec3 d = direction(pos);
    double gap = R - radius - pos.norm();
    return {gap, -d, point(d)};
}
static Mesh socket_patch(const Vec3& contact_normal, double patch_radius, int n) {
    Vec3 wall_dir = -normalize(contact_normal);
    auto [u, v, w] = basis_from_axis(wall_dir);
    Mesh m;
    m.name = "socket_patch";
    for (int j = 0; j <= n; ++j) {
        double b = -patch_radius + 2.0 * patch_radius * j / n;
        for (int i = 0; i <= n; ++i) {
            double a = -patch_radius + 2.0 * patch_radius * i / n;
            Vec3 d = normalize(w + (a / R) * u + (b / R) * v, w);
            m.vertices.push_back(point(d));
            m.normals.push_back(-d);
        }
    }
    auto vid = [&](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            m.faces.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
            m.faces.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
        }
    }
    return m;
}
}  // namespace Socket

class SphericalSocketGeometry final : public ContactGeometry {
public:
    std::string id() const override { return "spherical_socket"; }
    bool supports_sphere_response() const override { return true; }
    Mesh local_patch(const RigidBody& body, double sphere_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)body;
        (void)sphere_radius;
        (void)d_hat;
        return Socket::socket_patch(normal_hint, 0.076, 1);
    }
    ContactSampleCpp closest_sphere_contact(const RigidBody& body, double sphere_radius) const override {
        auto [gap, normal, point] = Socket::gap_normal(body.position, sphere_radius);
        return ContactSampleCpp{point, body.position - sphere_radius * normal, normal, gap};
    }
};

static ContactScene make_socket_scene() {
    constexpr double radius = 0.34;
    constexpr double mass = 0.85;
    Vec3 dir = normalize(Vec3(0.14, -0.06, -1.0));
    Vec3 n0 = -dir;

    ContactScene scene;
    scene.source = "spherical_socket_cpp";
    scene.dt = 2.0e-4;
    scene.gravity = Vec3(0.0, 0.0, -9.81);
    scene.body.mass = mass;
    scene.body.inertia = 0.4 * mass * radius * radius;
    scene.body.position = (Socket::R - radius - 0.002) * dir;
    scene.body.linear_velocity = Vec3(0.070, 0.045, 0.0);
    scene.body.linear_velocity -= scene.body.linear_velocity.dot(dir) * dir;
    scene.body.angular_velocity = n0.cross(scene.body.linear_velocity) / radius;
    scene.geometries.push_back(std::make_unique<SphericalSocketGeometry>());
    scene.geometries.push_back(std::make_unique<SphereGeometry>("sphere_patch", radius, 0.076, 1));
    scene.pairs.push_back(ContactPairSpec{
        "socket",
        0,
        1,
        0.012,
        ContactParams{0.003, 9000.0, 70.0, 0.40, 900.0, 2.0},
        n0,
    });
    scene.post_integrate = [](RigidBody& body) {
        constexpr double radius_local = 0.34;
        auto [gap, normal, point] = Socket::gap_normal(body.position, radius_local);
        (void)point;
        if (gap < -0.006) {
            Vec3 direction = -normal;
            body.position = (Socket::R - radius_local + 0.006) * direction;
            double out = body.linear_velocity.dot(direction);
            if (out > 0.0) body.linear_velocity -= out * direction;
        }
    };
    return scene;
}

static BenchResult run_socket(int steps, const std::string& output_dir = "") {
    ContactScene scene = make_socket_scene();
    return run_contact_scene(scene, steps, output_dir);
}

namespace BallJointPendulum {
constexpr double SOCKET_INNER_R = 0.42;
constexpr double SOCKET_OUTER_R = 0.50;
constexpr double SOCKET_OPEN_POLAR = 0.42;
constexpr double BALL_R = 0.34;
constexpr double BALL_MASS = 0.85;
constexpr double ROD_R = 0.035;
constexpr double ROD_LEN = 0.82;
constexpr double ROD_MASS = 0.25;
constexpr double PAYLOAD_R = 0.115;
constexpr double PAYLOAD_LEN = 0.22;
constexpr double PAYLOAD_MASS = 1.20;
constexpr double INITIAL_CLEARANCE = 2.0e-4;
constexpr double DT = 2.0e-4;
constexpr double PROJECTION_GAP_FLOOR = -6.0e-4;
constexpr double PROJECTION_GAP_CEILING = 1.2e-3;
constexpr double CONTACT_RELEASE_TOL = 1.2e-3;

struct SocketSample {
    double gap = 0.0;
    Vec3 normal = Vec3(0.0, 0.0, -1.0);
    Vec3 point = Vec3::Zero();
    int candidate_pairs = 0;
    double area_weight = 1.0;
    double response_weight = 1.0;
};

struct CompositeData {
    Vec3 axis = Vec3::UnitZ();
    Vec3 ball_center = Vec3::Zero();
    Vec3 com = Vec3::Zero();
    Vec3 ball_offset_body = Vec3::Zero();
    Mat3 inertia_body = Mat3::Identity();
    double total_mass = 1.0;
};

static Vec3 rod_axis() {
    const double tilt = 15.0 * PI / 180.0;
    const double azimuth = 25.0 * PI / 180.0;
    return normalize(Vec3(std::sin(tilt) * std::cos(azimuth), std::sin(tilt) * std::sin(azimuth), std::cos(tilt)));
}

static Vec3 initial_ball_center() {
    return Vec3(0.0, 0.0, -(SOCKET_INNER_R - BALL_R - INITIAL_CLEARANCE));
}

static Mat3 cylinder_inertia_matrix(double mass, double radius, double length, const Vec3& axis_in) {
    Vec3 a = normalize(axis_in);
    double i_axial = 0.5 * mass * radius * radius;
    double i_trans = (1.0 / 12.0) * mass * (3.0 * radius * radius + length * length);
    return i_trans * (Mat3::Identity() - a * a.transpose()) + i_axial * (a * a.transpose());
}

static Mat3 parallel_axis(const Mat3& inertia_com, double mass, const Vec3& offset) {
    return inertia_com + mass * (offset.squaredNorm() * Mat3::Identity() - offset * offset.transpose());
}

static CompositeData composite_body_data() {
    CompositeData data;
    data.axis = rod_axis();
    data.ball_center = initial_ball_center();
    Vec3 rod_center = data.ball_center + 0.5 * ROD_LEN * data.axis;
    Vec3 payload_center = data.ball_center + (ROD_LEN - 0.010 + 0.5 * PAYLOAD_LEN) * data.axis;
    data.total_mass = BALL_MASS + ROD_MASS + PAYLOAD_MASS;
    data.com = (BALL_MASS * data.ball_center + ROD_MASS * rod_center + PAYLOAD_MASS * payload_center) / data.total_mass;

    Mat3 ball_inertia = 0.4 * BALL_MASS * BALL_R * BALL_R * Mat3::Identity();
    Mat3 rod_inertia = cylinder_inertia_matrix(ROD_MASS, ROD_R, ROD_LEN, data.axis);
    Mat3 payload_inertia = cylinder_inertia_matrix(PAYLOAD_MASS, PAYLOAD_R, PAYLOAD_LEN, data.axis);
    data.inertia_body = parallel_axis(ball_inertia, BALL_MASS, data.ball_center - data.com) +
                        parallel_axis(rod_inertia, ROD_MASS, rod_center - data.com) +
                        parallel_axis(payload_inertia, PAYLOAD_MASS, payload_center - data.com);
    data.ball_offset_body = data.ball_center - data.com;
    return data;
}

static SocketSample rim_contact_sample(const Vec3& ball_center, double sphere_radius) {
    double open_z = sphere_radius * std::cos(SOCKET_OPEN_POLAR);
    double rim_radius = sphere_radius * std::sin(SOCKET_OPEN_POLAR);
    Vec2 xy(ball_center.x(), ball_center.y());
    double xy_norm = xy.norm();
    Vec2 rim_xy = xy_norm <= 1.0e-14 ? Vec2(rim_radius, 0.0) : (rim_radius / xy_norm) * xy;
    Vec3 rim_point(rim_xy.x(), rim_xy.y(), open_z);
    Vec3 offset = ball_center - rim_point;
    SocketSample sample;
    sample.gap = offset.norm() - BALL_R;
    sample.normal = normalize(offset, Vec3(0.0, 0.0, -1.0));
    sample.point = rim_point;
    sample.candidate_pairs = 1;
    return sample;
}

static SocketSample finite_socket_contact_sample(const Vec3& ball_center) {
    Vec3 rel = ball_center;
    double rho = rel.norm();
    Vec3 direction = normalize(rel, Vec3(0.0, 0.0, -1.0));
    double theta = std::acos(clamp(direction.z(), -1.0, 1.0));
    std::vector<SocketSample> samples;

    if (theta >= SOCKET_OPEN_POLAR) {
        SocketSample inner;
        inner.gap = SOCKET_INNER_R - BALL_R - rho;
        inner.normal = -direction;
        inner.point = SOCKET_INNER_R * direction;
        inner.candidate_pairs = 1;
        samples.push_back(inner);
    }
    samples.push_back(rim_contact_sample(ball_center, SOCKET_INNER_R));
    samples.push_back(rim_contact_sample(ball_center, SOCKET_OUTER_R));

    if (theta < SOCKET_OPEN_POLAR && rho >= SOCKET_OUTER_R) {
        SocketSample outer;
        outer.gap = rho - SOCKET_OUTER_R - BALL_R;
        outer.normal = direction;
        outer.point = SOCKET_OUTER_R * direction;
        outer.candidate_pairs = 1;
        samples.push_back(outer);
    }

    SocketSample best = *std::min_element(samples.begin(), samples.end(), [](const auto& a, const auto& b) {
        return a.gap < b.gap;
    });
    best.candidate_pairs = static_cast<int>(samples.size());
    return best;
}

static const OrientationInvariantSe3TricubicSdf& finite_socket_configuration_sdf() {
    static const OrientationInvariantSe3TricubicSdf sdf(
        Vec3(-0.20, -0.20, -0.18),
        Vec3(0.20, 0.20, 0.14),
        std::array<int, 3>{121, 121, 97},
        [](const Vec3& p) { return finite_socket_contact_sample(p).gap; });
    return sdf;
}

static SocketSample finite_socket_sdf_contact_sample(const OrientationInvariantSe3TricubicSdf& sdf,
                                                     const RigidConfiguration& ball_config) {
    const Vec3& ball_center = ball_config.position;
    SocketSample fallback = finite_socket_contact_sample(ball_center);
    Se3SdfSampleCpp s = sdf.sample(ball_config);
    if (!s.valid || !std::isfinite(s.phi) || !s.grad.allFinite()) return fallback;
    Vec3 normal = s.normal(fallback.normal);
    if (normal.dot(fallback.normal) < 0.0) normal = -normal;
    SocketSample out;
    out.gap = s.phi;
    out.normal = normal;
    out.point = ball_center - (BALL_R + out.gap) * out.normal;
    out.candidate_pairs = 1;
    return out;
}

static SocketSample finite_socket_sdf_contact_sample(const OrientationInvariantSe3TricubicSdf& sdf, const Vec3& ball_center) {
    return finite_socket_sdf_contact_sample(sdf, make_configuration(ball_center));
}

static std::vector<SocketSample> finite_socket_sdf_contact_patch(const OrientationInvariantSe3TricubicSdf& sdf,
                                                                 const RigidConfiguration& ball_config,
                                                                 const SocketSample& central,
                                                                 double patch_radius,
                                                                 int order,
                                                                 double outer_fraction = 0.10) {
    auto [t1, t2, nc] = basis_from_axis(central.normal);
    (void)nc;
    auto nodes = square_patch_quadrature(order, patch_radius, outer_fraction);
    double area_sum = 0.0;
    for (const auto& node : nodes) area_sum += std::get<1>(node);
    if (area_sum <= 0.0) area_sum = 1.0;

    std::vector<SocketSample> patch;
    patch.reserve(nodes.size());
    for (const auto& node : nodes) {
        Vec2 uv = std::get<0>(node);
        const double area_w = std::get<1>(node);
        RigidConfiguration shifted = ball_config;
        shifted.position += uv.x() * t1 + uv.y() * t2;
        SocketSample sample = finite_socket_sdf_contact_sample(sdf, shifted);
        sample.area_weight = area_w;
        sample.response_weight = area_w / area_sum;
        patch.push_back(sample);
    }
    return patch;
}

struct MeshDetectorSample {
    SocketSample sample;
    int contacts = 0;
    int candidate_pairs = 0;
    bool detector_contact = false;
};

static Mesh inner_socket_patch(const Vec3& contact_normal, double patch_radius, int n) {
    Vec3 wall_dir = -normalize(contact_normal, Vec3(0.0, 0.0, 1.0));
    auto [u, v, w] = basis_from_axis(wall_dir);
    Mesh m;
    m.name = "deep_socket_inner_mesh";
    for (int j = 0; j <= n; ++j) {
        double b = -patch_radius + 2.0 * patch_radius * j / n;
        for (int i = 0; i <= n; ++i) {
            double a = -patch_radius + 2.0 * patch_radius * i / n;
            Vec3 d = normalize(w + (a / SOCKET_INNER_R) * u + (b / SOCKET_INNER_R) * v, w);
            m.vertices.push_back(SOCKET_INNER_R * d);
            m.normals.push_back(-d);
        }
    }
    auto vid = [&](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            m.faces.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
            m.faces.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
        }
    }
    return m;
}

static MeshDetectorSample finite_socket_mesh_detector_sample(const Vec3& ball_center,
                                                             double d_hat,
                                                             double patch_radius,
                                                             int socket_resolution,
                                                             int sphere_resolution) {
    MeshDetectorSample out;
    out.sample = finite_socket_contact_sample(ball_center);
    Mesh socket_mesh = inner_socket_patch(out.sample.normal, patch_radius, socket_resolution);
    Mesh ball_mesh = sphere_patch(ball_center, BALL_R, out.sample.normal, patch_radius, sphere_resolution, "deep_ball_joint_sphere_mesh");
    auto [contacts, stats] = detect_curved(socket_mesh, ball_mesh, d_hat);
    out.contacts = stats.contacts;
    out.candidate_pairs = stats.candidate_pairs;
    out.sample.candidate_pairs = stats.candidate_pairs;
    if (!contacts.empty()) {
        ContactSampleCpp selected = orient_sample(select_contact(contacts, ball_center), ball_center);
        out.sample.point = selected.point_a;
        out.sample.normal = normalize(selected.normal, out.sample.normal);
        out.sample.gap = (ball_center - selected.point_a).dot(out.sample.normal) - BALL_R;
        out.detector_contact = true;
    }
    return out;
}

struct PendulumTraceWriter {
    std::ofstream file;
    std::string source;
    explicit PendulumTraceWriter(const std::string& output_dir,
                                 std::string stem = "deep_ball_joint_pendulum_cpp",
                                 std::string source_name = "deep_ball_joint_pendulum_cpp")
        : source(std::move(source_name)) {
        if (output_dir.empty()) return;
        std::filesystem::create_directories(output_dir);
        file.open((std::filesystem::path(output_dir) / (stem + ".csv")).string());
        file << std::setprecision(17);
        file << "source,step,time,x,y,z,vx,vy,vz,ax,ay,az,wx,wy,wz,gap,normal_x,normal_y,normal_z,"
                "contact_force_x,contact_force_y,contact_force_z,contact_force,contacts,candidate_pairs\n";
    }
    void write(int step,
               double time,
               const Vec3& pos,
               const Vec3& vel,
               const Vec3& acc,
               const Vec3& omega,
               const SocketSample& sample,
               const Vec3& contact_force,
               int contacts,
               int candidate_pairs) {
        if (!file) return;
        file << source << ',' << step << ',' << time << ','
             << pos.x() << ',' << pos.y() << ',' << pos.z() << ','
             << vel.x() << ',' << vel.y() << ',' << vel.z() << ','
             << acc.x() << ',' << acc.y() << ',' << acc.z() << ','
             << omega.x() << ',' << omega.y() << ',' << omega.z() << ','
             << sample.gap << ','
             << sample.normal.x() << ',' << sample.normal.y() << ',' << sample.normal.z() << ','
             << contact_force.x() << ',' << contact_force.y() << ',' << contact_force.z() << ','
             << contact_force.norm() << ',' << contacts << ',' << candidate_pairs << '\n';
    }
};

}  // namespace BallJointPendulum

static BenchResult run_deep_ball_joint_pendulum(int steps, const std::string& output_dir = "") {
    using namespace BallJointPendulum;
    CompositeData data = composite_body_data();
    FullRigidBody body;
    body.position = data.com;
    body.rotation = Mat3::Identity();
    body.linear_velocity = Vec3::Zero();
    body.angular_velocity = Vec3::Zero();
    body.mass = data.total_mass;
    body.inertia_body = data.inertia_body;
    body.inertia_body_inv = data.inertia_body.inverse();

    ContactParams params;
    params.d_hat = 0.0;
    params.kn = 2.0e5;
    params.cn = 650.0;
    params.mu = 0.35;
    params.kt = 3.0e5;
    params.ct = 300.0;
    params.penetration_only = true;
    params.release_tol = CONTACT_RELEASE_TOL;
    FrictionState friction_state;
    Vec3 gravity(0.0, 0.0, -9.81);
    bool contact_active = finite_socket_contact_sample(body.position + body.rotation * data.ball_offset_body).gap <= CONTACT_RELEASE_TOL;

    PendulumTraceWriter trace(output_dir);
    int total_contacts = 0;
    int total_pairs = 0;
    Vec3 final_ball_center = data.ball_center;
    Vec3 final_ball_velocity = Vec3::Zero();
    double final_gap = 0.0;

    auto t0 = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) {
        Vec3 ball_center = body.position + body.rotation * data.ball_offset_body;
        SocketSample sample = finite_socket_contact_sample(ball_center);
        Vec3 contact_point = ball_center - BALL_R * sample.normal;
        ContactEval contact = evaluate_contact_full(body, contact_point, sample.normal, sample.gap, DT, params, friction_state);
        Vec3 force = body.mass * gravity + contact.total_force;
        body.integrate(force, contact.torque, DT);

        Vec3 ball_center_after = body.position + body.rotation * data.ball_offset_body;
        SocketSample after_sample = finite_socket_contact_sample(ball_center_after);
        if (after_sample.gap < PROJECTION_GAP_FLOOR) {
            Vec3 correction = (PROJECTION_GAP_FLOOR - after_sample.gap) * after_sample.normal;
            body.position += correction;
            ball_center_after += correction;
            after_sample = finite_socket_contact_sample(ball_center_after);
            double vn = body.point_velocity(ball_center_after).dot(after_sample.normal);
            if (vn < 0.0) body.linear_velocity -= vn * after_sample.normal;
        } else if (contact_active && after_sample.gap > PROJECTION_GAP_CEILING) {
            Vec3 correction = -(after_sample.gap - PROJECTION_GAP_CEILING) * after_sample.normal;
            body.position += correction;
            ball_center_after += correction;
            after_sample = finite_socket_contact_sample(ball_center_after);
            double vn = body.point_velocity(ball_center_after).dot(after_sample.normal);
            if (vn > 0.0) body.linear_velocity -= vn * after_sample.normal;
        }

        contact_active = after_sample.gap <= CONTACT_RELEASE_TOL;
        Vec3 ball_velocity = body.point_velocity(ball_center_after);
        Vec3 output_contact_point = ball_center_after - BALL_R * after_sample.normal;
        Vec3 total_contact_torque = (output_contact_point - body.position).cross(contact.total_force);
        Vec3 force_for_accel = body.mass * gravity + contact.total_force;
        Vec3 ball_arm_after = body.rotation * data.ball_offset_body;
        Vec3 angular_accel_after = body.angular_acceleration(total_contact_torque);
        Vec3 ball_accel = force_for_accel / body.mass + angular_accel_after.cross(ball_arm_after) +
                          body.angular_velocity.cross(body.angular_velocity.cross(ball_arm_after));

        total_contacts += contact_active ? 1 : 0;
        total_pairs += after_sample.candidate_pairs;
        final_ball_center = ball_center_after;
        final_ball_velocity = ball_velocity;
        final_gap = after_sample.gap;
        trace.write(step, (step + 1) * DT, ball_center_after, ball_velocity, ball_accel, body.angular_velocity, after_sample,
                    contact.total_force, contact_active ? 1 : 0, after_sample.candidate_pairs);
    }
    auto t1 = std::chrono::steady_clock::now();
    return {
        "deep_ball_joint_pendulum_cpp",
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        steps,
        total_contacts,
        total_pairs,
        final_ball_center,
        final_ball_velocity,
        final_gap,
    };
}

static BenchResult run_deep_ball_joint_pendulum_mesh_detector(int steps, const std::string& output_dir = "") {
    using namespace BallJointPendulum;
    constexpr double DETECTION_D_HAT = 0.012;
    constexpr double PATCH_RADIUS = 0.090;
    constexpr int SOCKET_RESOLUTION = 3;
    constexpr int SPHERE_RESOLUTION = 3;
    const OrientationInvariantSe3TricubicSdf& socket_sdf = finite_socket_configuration_sdf();

    CompositeData data = composite_body_data();
    FullRigidBody body;
    body.position = data.com;
    body.rotation = Mat3::Identity();
    body.linear_velocity = Vec3::Zero();
    body.angular_velocity = Vec3::Zero();
    body.mass = data.total_mass;
    body.inertia_body = data.inertia_body;
    body.inertia_body_inv = data.inertia_body.inverse();

    ContactParams params;
    params.d_hat = 0.0;
    params.kn = 2.0e5;
    params.cn = 650.0;
    params.mu = 0.35;
    params.kt = 3.0e5;
    params.ct = 300.0;
    params.penetration_only = true;
    params.release_tol = CONTACT_RELEASE_TOL;
    SurfaceFrictionState friction_state;
    Vec3 gravity(0.0, 0.0, -9.81);
    bool contact_active =
        finite_socket_sdf_contact_sample(socket_sdf, make_configuration(body.position + body.rotation * data.ball_offset_body, body.rotation))
            .gap <= CONTACT_RELEASE_TOL;

    PendulumTraceWriter trace(output_dir, "deep_ball_joint_pendulum_mesh_cpp", "deep_ball_joint_pendulum_mesh_cpp");
    int total_contacts = 0;
    int total_pairs = 0;
    Vec3 final_ball_center = data.ball_center;
    Vec3 final_ball_velocity = Vec3::Zero();
    double final_gap = 0.0;

    auto t0 = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) {
        Vec3 ball_center = body.position + body.rotation * data.ball_offset_body;
        MeshDetectorSample detected =
            finite_socket_mesh_detector_sample(ball_center, DETECTION_D_HAT, PATCH_RADIUS, SOCKET_RESOLUTION, SPHERE_RESOLUTION);
        // The mesh detector supplies candidate/contact work; dense tricubic SDF refinement gives the signed gap and force normal.
        SocketSample force_sample = finite_socket_sdf_contact_sample(socket_sdf, make_configuration(ball_center, body.rotation));
        force_sample.candidate_pairs = detected.candidate_pairs;
        if (!detected.detector_contact) {
            force_sample.gap = std::max(force_sample.gap, CONTACT_RELEASE_TOL + 1.0e-6);
        }
        std::vector<SocketSample> patch =
            finite_socket_sdf_contact_patch(socket_sdf, make_configuration(ball_center, body.rotation), force_sample, 0.0040, 3, 1.0e-3);
        ContactEval contact;
        for (size_t qi = 0; qi < patch.size(); ++qi) {
            SocketSample sample = patch[qi];
            if (!detected.detector_contact) {
                sample.gap = std::max(sample.gap, CONTACT_RELEASE_TOL + 1.0e-6);
            }
            Vec3 contact_point = sample.point + sample.gap * sample.normal;
            ContactParams weighted_params = scaled_contact_params(params, sample.response_weight);
            ContactEval c = evaluate_contact_full(body,
                                                  contact_point,
                                                  sample.normal,
                                                  sample.gap,
                                                  DT,
                                                  weighted_params,
                                                  friction_state.at(qi, patch.size()));
            contact.total_force += c.total_force;
            contact.torque += c.torque;
        }
        Vec3 force = body.mass * gravity + contact.total_force;
        body.integrate(force, contact.torque, DT);

        Vec3 ball_center_after = body.position + body.rotation * data.ball_offset_body;
        SocketSample after_sample = finite_socket_sdf_contact_sample(socket_sdf, make_configuration(ball_center_after, body.rotation));
        if (after_sample.gap < PROJECTION_GAP_FLOOR) {
            Vec3 correction = (PROJECTION_GAP_FLOOR - after_sample.gap) * after_sample.normal;
            body.position += correction;
            ball_center_after += correction;
            after_sample = finite_socket_sdf_contact_sample(socket_sdf, make_configuration(ball_center_after, body.rotation));
            double vn = body.point_velocity(ball_center_after).dot(after_sample.normal);
            if (vn < 0.0) body.linear_velocity -= vn * after_sample.normal;
        } else if (contact_active && after_sample.gap > PROJECTION_GAP_CEILING) {
            Vec3 correction = -(after_sample.gap - PROJECTION_GAP_CEILING) * after_sample.normal;
            body.position += correction;
            ball_center_after += correction;
            after_sample = finite_socket_sdf_contact_sample(socket_sdf, make_configuration(ball_center_after, body.rotation));
            double vn = body.point_velocity(ball_center_after).dot(after_sample.normal);
            if (vn > 0.0) body.linear_velocity -= vn * after_sample.normal;
        }

        contact_active = after_sample.gap <= CONTACT_RELEASE_TOL;
        Vec3 ball_velocity = body.point_velocity(ball_center_after);
        Vec3 output_contact_point = ball_center_after - BALL_R * after_sample.normal;
        Vec3 total_contact_torque = (output_contact_point - body.position).cross(contact.total_force);
        Vec3 force_for_accel = body.mass * gravity + contact.total_force;
        Vec3 ball_arm_after = body.rotation * data.ball_offset_body;
        Vec3 angular_accel_after = body.angular_acceleration(total_contact_torque);
        Vec3 ball_accel = force_for_accel / body.mass + angular_accel_after.cross(ball_arm_after) +
                          body.angular_velocity.cross(body.angular_velocity.cross(ball_arm_after));

        total_contacts += detected.contacts;
        total_pairs += detected.candidate_pairs;
        final_ball_center = ball_center_after;
        final_ball_velocity = ball_velocity;
        final_gap = after_sample.gap;
        after_sample.candidate_pairs = detected.candidate_pairs;
        trace.write(step, (step + 1) * DT, ball_center_after, ball_velocity, ball_accel, body.angular_velocity, after_sample,
                    contact.total_force, detected.contacts, detected.candidate_pairs);
    }
    auto t1 = std::chrono::steady_clock::now();
    return {
        "deep_ball_joint_pendulum_mesh_cpp",
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        steps,
        total_contacts,
        total_pairs,
        final_ball_center,
        final_ball_velocity,
        final_gap,
    };
}

namespace Bearing {
constexpr double TRACK_R = 1.0;
constexpr double GROOVE_R = 0.161;
constexpr double PRELOAD = 2.0e-4;
constexpr double BALL_R = GROOVE_R + PRELOAD;
constexpr double INNER_RING_OMEGA = 0.036;
constexpr double INNER_PHI = PI;
constexpr double OUTER_PHI = 0.0;
static std::tuple<Vec3, Vec3, Vec3> basis(double theta) {
    Vec3 er(std::cos(theta), std::sin(theta), 0.0);
    Vec3 et(-std::sin(theta), std::cos(theta), 0.0);
    return {er, et, Vec3(0.0, 0.0, 1.0)};
}
static double theta_from_pos(const Vec3& p) { return std::atan2(p.y(), p.x()); }
static Vec3 point(double theta, double phi) {
    auto [er, et, ez] = basis(theta);
    return (TRACK_R + GROOVE_R * std::cos(phi)) * er + GROOVE_R * std::sin(phi) * ez;
}
static Vec3 normal(double theta, double phi) {
    auto [er, et, ez] = basis(theta);
    return normalize(-(std::cos(phi) * er + std::sin(phi) * ez));
}
static double side_phi(const std::string& side) { return side == "inner" ? INNER_PHI : OUTER_PHI; }
static std::tuple<double, Vec3, Vec3> side_gap_normal(const Vec3& pos, const std::string& side) {
    double th = theta_from_pos(pos);
    Vec3 p = point(th, side_phi(side));
    Vec3 n = normalize(pos - p, normal(th, side_phi(side)));
    return {(pos - p).norm() - BALL_R, n, p};
}

static const OrientationInvariantSe3TricubicSdf& side_configuration_sdf(const std::string& side) {
    static const OrientationInvariantSe3TricubicSdf inner_sdf(
        Vec3(-1.24, -1.24, -0.28),
        Vec3(1.24, 1.24, 0.28),
        std::array<int, 3>{181, 181, 81},
        [](const Vec3& p) { return std::get<0>(side_gap_normal(p, "inner")); });
    static const OrientationInvariantSe3TricubicSdf outer_sdf(
        Vec3(-1.24, -1.24, -0.28),
        Vec3(1.24, 1.24, 0.28),
        std::array<int, 3>{181, 181, 81},
        [](const Vec3& p) { return std::get<0>(side_gap_normal(p, "outer")); });
    return side == "inner" ? inner_sdf : outer_sdf;
}

static ContactSampleCpp side_sdf_contact(const Vec3& center, const std::string& side, double sphere_radius) {
    auto [fallback_gap, fallback_normal, fallback_point] = side_gap_normal(center, side);
    Se3SdfSampleCpp s = side_configuration_sdf(side).sample(make_configuration(center));
    if (!s.valid || !std::isfinite(s.phi) || !s.grad.allFinite()) {
        return ContactSampleCpp{fallback_point, center - sphere_radius * fallback_normal, fallback_normal, fallback_gap};
    }
    Vec3 normal = s.normal(fallback_normal);
    if (normal.dot(fallback_normal) < 0.0) normal = -normal;
    double gap = s.phi;
    Vec3 surface_point = center - (sphere_radius + gap) * normal;
    return ContactSampleCpp{surface_point, center - sphere_radius * normal, normal, gap};
}

static Mesh raceway_patch(double theta, const std::string& side) {
    Mesh m;
    m.name = side + "_raceway_patch";
    double pc = side_phi(side);
    constexpr int ntheta = 2;
    constexpr int nphi = 2;
    for (int i = 0; i <= ntheta; ++i) {
        double th = theta - 0.075 + 2.0 * 0.075 * i / ntheta;
        for (int j = 0; j <= nphi; ++j) {
            double ph = pc - 0.42 + 2.0 * 0.42 * j / nphi;
            m.vertices.push_back(point(th, ph));
            m.normals.push_back(normal(th, ph));
        }
    }
    auto vid = [&](int i, int j) { return i * (nphi + 1) + j; };
    for (int i = 0; i < ntheta; ++i) {
        for (int j = 0; j < nphi; ++j) {
            m.faces.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
            m.faces.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
        }
    }
    return m;
}
}  // namespace Bearing

class BearingRacewayGeometry final : public ContactGeometry {
public:
    explicit BearingRacewayGeometry(std::string side) : side_(std::move(side)) {}

    std::string id() const override { return "bearing_" + side_; }
    bool supports_sphere_response() const override { return true; }
    Mesh local_patch(const RigidBody& body, double sphere_radius, const Vec3& normal_hint, double d_hat) const override {
        (void)sphere_radius;
        (void)normal_hint;
        (void)d_hat;
        return Bearing::raceway_patch(Bearing::theta_from_pos(body.position), side_);
    }
    ContactSampleCpp closest_sphere_contact(const RigidBody& body, double sphere_radius) const override {
        return Bearing::side_sdf_contact(body.position, side_, sphere_radius);
    }
    Vec3 surface_velocity(const Vec3& world_point) const override {
        if (side_ == "inner") return Vec3(0.0, 0.0, Bearing::INNER_RING_OMEGA).cross(world_point);
        return Vec3::Zero();
    }

private:
    std::string side_;
};

static ContactScene make_bearing_scene() {
    constexpr double mass = 0.85;
    auto [er, et, ez] = Bearing::basis(0.24);

    ContactScene scene;
    scene.source = "bearing_rotating_inner_cpp";
    scene.dt = 5.0e-4;
    scene.gravity = Vec3::Zero();
    scene.body.mass = mass;
    scene.body.inertia = 0.4 * mass * Bearing::BALL_R * Bearing::BALL_R;
    scene.body.position = Bearing::TRACK_R * er;
    scene.body.linear_velocity = 0.015 * et;
    scene.body.angular_velocity = -(0.015 / Bearing::BALL_R) * ez;
    scene.geometries.push_back(std::make_unique<BearingRacewayGeometry>("inner"));
    scene.geometries.push_back(std::make_unique<BearingRacewayGeometry>("outer"));
    scene.geometries.push_back(std::make_unique<SphereGeometry>("sphere_patch", Bearing::BALL_R, 0.060, 2));

    auto [inner_gap, inner_normal, inner_point] = Bearing::side_gap_normal(scene.body.position, "inner");
    auto [outer_gap, outer_normal, outer_point] = Bearing::side_gap_normal(scene.body.position, "outer");
    (void)inner_gap;
    (void)inner_point;
    (void)outer_gap;
    (void)outer_point;
    ContactParams params{0.003, 700.0, 28.0, 0.08, 2.0, 0.010, true};
    ContactPairSpec inner_pair{"bearing_inner", 0, 2, 0.010, params, inner_normal};
    inner_pair.response_field = std::make_shared<FunctionalContactResponseField>(
        [](const RigidConfiguration& q) { return Bearing::side_configuration_sdf("inner").sample(q); });
    inner_pair.surface_patch_radius = 0.0040;
    inner_pair.surface_quadrature_order = 2;
    inner_pair.surface_outer_fraction = 1.0e-3;
    inner_pair.detector_stride = 2;
    ContactPairSpec outer_pair{"bearing_outer", 1, 2, 0.010, params, outer_normal};
    outer_pair.response_field = std::make_shared<FunctionalContactResponseField>(
        [](const RigidConfiguration& q) { return Bearing::side_configuration_sdf("outer").sample(q); });
    outer_pair.surface_patch_radius = 0.0040;
    outer_pair.surface_quadrature_order = 2;
    outer_pair.surface_outer_fraction = 1.0e-3;
    outer_pair.detector_stride = 2;
    scene.pairs.push_back(inner_pair);
    scene.pairs.push_back(outer_pair);
    return scene;
}

static BenchResult run_bearing(int steps, const std::string& output_dir = "") {
    (void)Bearing::side_configuration_sdf("inner");
    (void)Bearing::side_configuration_sdf("outer");
    ContactScene scene = make_bearing_scene();
    return run_contact_scene(scene, steps, output_dir);
}

static void print_result(const BenchResult& r) {
    std::cout << "  {\n";
    std::cout << "    \"case\": \"" << r.name << "\",\n";
    std::cout << "    \"steps\": " << r.steps << ",\n";
    std::cout << "    \"elapsed_ms\": " << r.elapsed_ms << ",\n";
    std::cout << "    \"us_per_step\": " << (1000.0 * r.elapsed_ms / std::max(1, r.steps)) << ",\n";
    std::cout << "    \"avg_contacts\": " << (static_cast<double>(r.contacts) / std::max(1, r.steps)) << ",\n";
    std::cout << "    \"avg_candidate_pairs\": " << (static_cast<double>(r.pairs) / std::max(1, r.steps)) << ",\n";
    std::cout << "    \"final_position\": [" << r.final_position.x() << ", " << r.final_position.y() << ", " << r.final_position.z() << "],\n";
    std::cout << "    \"final_velocity\": [" << r.final_velocity.x() << ", " << r.final_velocity.y() << ", " << r.final_velocity.z() << "],\n";
    std::cout << "    \"final_gap\": " << r.final_gap << "\n";
    std::cout << "  }";
}

int main(int argc, char** argv) {
    int steps = 1000;
    if (argc >= 2) steps = std::max(1, std::atoi(argv[1]));
    std::string output_dir;
    std::string selected_case = "all";
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--out-dir" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--case" && i + 1 < argc) {
            selected_case = argv[++i];
        } else if (arg == "--guide-target-gap" && i + 1 < argc) {
            g_guide_target_gap = std::atof(argv[++i]);
        } else if (arg == "--guide-mu" && i + 1 < argc) {
            g_guide_mu = std::atof(argv[++i]);
        } else if (arg == "all" || arg == "all_mesh" || arg == "guide" || arg == "guide_slot" || arg == "socket" ||
                   arg == "spherical_socket" || arg == "ball_joint" || arg == "deep_ball_joint_pendulum" ||
                   arg == "ball_joint_mesh" || arg == "deep_ball_joint_pendulum_mesh" || arg == "bearing" ||
                   arg == "bearing_rotating_inner" || arg == "generic_mesh" || arg == "generic_mesh_smoke") {
            selected_case = arg;
        } else if (output_dir.empty()) {
            output_dir = arg;
        }
    }
    std::vector<BenchResult> results;
    if (selected_case == "all" || selected_case == "all_mesh" || selected_case == "guide" || selected_case == "guide_slot") {
        results.push_back(run_guide(steps, output_dir));
    }
    if (selected_case == "generic_mesh" || selected_case == "generic_mesh_smoke") {
        results.push_back(run_generic_mesh_smoke(steps, output_dir));
    }
    if (selected_case == "socket" || selected_case == "spherical_socket") {
        results.push_back(run_socket(steps, output_dir));
    }
    if (selected_case == "ball_joint" || selected_case == "deep_ball_joint_pendulum") {
        results.push_back(run_deep_ball_joint_pendulum(steps, output_dir));
    }
    if (selected_case == "all" || selected_case == "all_mesh" || selected_case == "ball_joint_mesh" ||
        selected_case == "deep_ball_joint_pendulum_mesh") {
        results.push_back(run_deep_ball_joint_pendulum_mesh_detector(steps, output_dir));
    }
    if (selected_case == "all" || selected_case == "all_mesh" || selected_case == "bearing" || selected_case == "bearing_rotating_inner") {
        results.push_back(run_bearing(steps, output_dir));
    }
    if (results.empty()) {
        std::cerr << "unknown case: " << selected_case << "\n";
        return 2;
    }
    std::cout << "{\n  \"backend\": \"calg_cpp_surface_patch_sdf_contact\",\n  \"results\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        print_result(results[i]);
        std::cout << (i + 1 == results.size() ? "\n" : ",\n");
    }
    std::cout << "  ]\n}\n";
    return 0;
}
