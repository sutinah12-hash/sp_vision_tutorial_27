#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace nav_tracking {
using Vec2 = Eigen::Vector2d;

inline Vec2 bounded(const Vec2 & v, double limit) {
  if (!v.allFinite() || !std::isfinite(limit) || limit < 0.0) return Vec2::Zero();
  const double n = v.norm();
  return n > limit && n > 1e-12 ? Vec2(v * (limit / n)) : v;
}

inline Vec2 worldToBody(const Vec2 & v, double yaw) {
  return {std::cos(yaw) * v.x() + std::sin(yaw) * v.y(),
          -std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};
}

// Arc-length interpolation and a curvature/braking-limited speed profile.
class ReferencePath {
 public:
  std::vector<Vec2> points;
  std::vector<double> arc, speeds;

  void set(const std::vector<Vec2> & input, double cruise, double brake, double lateral) {
    points.clear(); arc.clear(); speeds.clear();
    for (const auto & p : input) {
      if (!p.allFinite()) { points.clear(); return; }
      if (points.empty() || (p - points.back()).norm() > 1e-5) points.push_back(p);
    }
    if (points.empty()) return;
    arc.push_back(0.0);
    for (std::size_t i = 1; i < points.size(); ++i)
      arc.push_back(arc.back() + (points[i] - points[i-1]).norm());
    speeds.assign(points.size(), cruise);
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
      const Vec2 a = at(std::max(0.0, arc[i] - 0.20));
      const Vec2 b = at(std::min(length(), arc[i] + 0.20));
      const Vec2 u = points[i] - a, v = b - points[i];
      if (u.norm() > 1e-5 && v.norm() > 1e-5) {
        const double angle = std::acos(std::clamp(u.dot(v) / (u.norm()*v.norm()), -1.0, 1.0));
        const double curvature = angle / std::max(0.05, (u.norm()+v.norm())*0.5);
        speeds[i] = std::min(cruise, std::sqrt(lateral / std::max(curvature, 1e-4)));
      }
    }
    speeds.back() = 0.0;
    for (std::size_t i = speeds.size()-1; i > 0; --i)
      speeds[i-1] = std::min(speeds[i-1], std::sqrt(speeds[i]*speeds[i] + 2.0*brake*(arc[i]-arc[i-1])));
    for (std::size_t i = 1; i < speeds.size(); ++i)
      speeds[i] = std::min(speeds[i], std::sqrt(speeds[i-1]*speeds[i-1] + 2.0*brake*(arc[i]-arc[i-1])));
  }

  double length() const { return arc.empty() ? 0.0 : arc.back(); }

  Vec2 at(double s) const {
    if (points.empty()) return Vec2::Zero();
    if (points.size() == 1 || s <= 0.0) return points.front();
    if (s >= length()) return points.back();
    const auto i = static_cast<std::size_t>(std::upper_bound(arc.begin(), arc.end(), s)-arc.begin());
    const double t = (s-arc[i-1]) / (arc[i]-arc[i-1]);
    return points[i-1]*(1.0-t) + points[i]*t;
  }

  double speed(double s) const {
    if (speeds.size() < 2 || s >= length()) return 0.0;
    if (s <= 0.0) return speeds.front();
    const auto i = static_cast<std::size_t>(std::upper_bound(arc.begin(), arc.end(), s)-arc.begin());
    const double t = (s-arc[i-1]) / (arc[i]-arc[i-1]);
    return speeds[i-1]*(1.0-t) + speeds[i]*t;
  }

  Vec2 tangent(double s) const {
    Vec2 d = at(std::min(length(), s+0.04)) - at(std::max(0.0, s-0.04));
    return d.norm() > 1e-9 ? Vec2(d.normalized()) : Vec2::Zero();
  }

  double nearest(const Vec2 & p) const {
    double best = 1e100, best_s = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const Vec2 d = points[i]-points[i-1];
      const double t = std::clamp((p-points[i-1]).dot(d)/d.squaredNorm(), 0.0, 1.0);
      const double error = (p-(points[i-1]+t*d)).squaredNorm();
      if (error < best) { best=error; best_s=arc[i-1]+t*d.norm(); }
    }
    return best_s;
  }

  void preview(const Vec2 & position, const Vec2 & velocity, int n, double dt,
               Eigen::MatrixXd & pos_ref, Eigen::MatrixXd & vel_ref) const {
    pos_ref.resize(n, 2); vel_ref.resize(n, 2);
    double s = nearest(position);
    double v = std::min(velocity.norm(), speed(s));
    for (int i=0; i<n; ++i) {
      v = std::min(speed(s), v + 0.9*dt);
      s = std::min(length(), s + std::max(0.04, v)*dt);
      pos_ref.row(i) = at(s).transpose();
      vel_ref.row(i) = (tangent(s)*speed(s)).transpose();
    }
  }
};

// Condensed convex MPC: v[k+1]=alpha*v[k]+(1-alpha)*u[k],
// p[k+1]=p[k]+dt*v[k+1]. The optimization variables are world-frame velocities.
// Minimize position/velocity errors, control effort and command differences.
// Every horizon command is projected onto a Euclidean speed disk (no extra QP dependency).
class VelocityMpc {
 public:
  int horizon = 24, iterations = 100;
  double dt = 0.08, tau = 0.23, speed_limit = 1.35;
  double position_weight = 24.0, velocity_weight = 1.5, change_weight = 0.7;
  Eigen::MatrixXd prediction;

  void initialize() {
    if (horizon < 2 || horizon > 100 || dt <= 0 || tau <= 0 || speed_limit <= 0 ||
        iterations < 1 || position_weight <= 0 || velocity_weight < 0 || change_weight < 0)
      throw std::invalid_argument("Invalid MPC horizon, dynamics or weights");
    alpha_ = std::exp(-dt/tau);
    p_ = Eigen::MatrixXd::Zero(horizon,horizon);
    v_ = p_; Eigen::MatrixXd d = Eigen::MatrixXd::Identity(horizon,horizon);
    for (int i=0; i<horizon; ++i) {
      if (i) d(i,i-1)=-1.0;
      for (int j=0; j<=i; ++j) {
        v_(i,j)=(1.0-alpha_)*std::pow(alpha_,i-j);
        p_(i,j)=dt*(1.0-std::pow(alpha_,i-j+1));
      }
    }
    q_ = Eigen::VectorXd::Constant(horizon,position_weight);
    q_[horizon-1] *= 3.0;
    h_ = p_.transpose()*q_.asDiagonal()*p_ + velocity_weight*v_.transpose()*v_ +
         change_weight*d.transpose()*d + 0.02*Eigen::MatrixXd::Identity(horizon,horizon);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(h_);
    step_=1.0/eig.eigenvalues().maxCoeff();
    u_=Eigen::MatrixXd::Zero(horizon,2);
  }

  void reset() { u_.setZero(); }

  Vec2 solve(const Vec2 & position, const Vec2 & velocity, const Vec2 & previous,
             const Eigen::MatrixXd & pos_ref, const Eigen::MatrixXd & vel_ref,
             const Eigen::MatrixXd & pending=Eigen::MatrixXd()) {
    if (!position.allFinite() || !velocity.allFinite() || !previous.allFinite() || !pos_ref.allFinite() ||
        !vel_ref.allFinite() || pos_ref.rows()!=horizon || vel_ref.rows()!=horizon ||
        pos_ref.cols()!=2 || vel_ref.cols()!=2 ||
        pending.rows()>=horizon || (pending.rows()>0 && pending.cols()!=2) || !pending.allFinite())
      return Vec2::Zero();
    Eigen::MatrixXd free_p(horizon,2), free_v(horizon,2);
    for (int i=0; i<horizon; ++i) {
      const double power=std::pow(alpha_,i+1);
      free_v.row(i)=(velocity*power).transpose();
      free_p.row(i)=(position+velocity*(dt*alpha_*(1.0-power)/(1.0-alpha_))).transpose();
    }
    Eigen::MatrixXd f=p_.transpose()*q_.asDiagonal()*(free_p-pos_ref) +
                       velocity_weight*v_.transpose()*(free_v-vel_ref);
    f.row(0)-=change_weight*previous.transpose();
    Eigen::MatrixXd y=u_; double t=1.0;
    for (int iteration=0; iteration<iterations; ++iteration) {
      Eigen::MatrixXd next=y-step_*(h_*y+f);
      for (int i=0; i<horizon; ++i)
        next.row(i)=bounded(Vec2(next.row(i).transpose()),speed_limit).transpose();
      // Commands already in the actuator delay queue are fixed, not decision variables.
      for (int i=0; i<pending.rows(); ++i) next.row(i)=pending.row(i);
      const double new_t=(1.0+std::sqrt(1.0+4.0*t*t))*0.5;
      y=next+((t-1.0)/new_t)*(next-u_);
      const double change=(next-u_).norm(); u_=next; t=new_t;
      if (change<1e-5) break;
    }
    prediction=free_p+p_*u_;
    return Vec2(u_.row(pending.rows()).transpose());
  }

 private:
  double alpha_=0.0, step_=0.0;
  Eigen::MatrixXd p_,v_,h_,u_;
  Eigen::VectorXd q_;
};
}  // namespace nav_tracking
