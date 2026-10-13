#include "extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace tools
{
namespace
{
bool covariance_ok(const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == 0 || matrix.rows() != matrix.cols() || !matrix.allFinite()) return false;
  if (!matrix.isApprox(matrix.transpose(), 1e-9)) return false;
  // Q is often singular (rank-one acceleration blocks); an LDLT failure does not
  // imply that a semidefinite covariance is invalid. Check eigenvalues explicitly.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(matrix, Eigen::EigenvaluesOnly);
  return spectrum.info() == Eigen::Success && spectrum.eigenvalues().minCoeff() >= -1e-10;
}
}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.size(), x0.size())), x_add(std::move(add))
{
  if (x.size() == 0 || !x.allFinite() || P.rows() != x.size() || !covariance_ok(P) || !x_add)
    throw std::invalid_argument("EKF requires a finite state and a symmetric PSD covariance");
  data["nis"] = 0.0;
  data["nis_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & state) { return (F * state).eval(); });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  if (x.size() == 0 || F.rows() != x.size() || F.cols() != x.size() ||
      Q.rows() != x.size() || !F.allFinite() || !covariance_ok(Q))
    throw std::invalid_argument("EKF prediction matrix dimension or covariance is invalid");
  Eigen::VectorXd predicted_x = f(x);
  Eigen::MatrixXd predicted_P = F * P * F.transpose() + Q;
  if (predicted_x.size() != x.size() || !predicted_x.allFinite() || !predicted_P.allFinite())
    throw std::runtime_error("EKF prediction produced a non-finite state or covariance");
  x = std::move(predicted_x);
  P = 0.5 * (predicted_P + predicted_P.transpose()).eval();
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & state) { return (H * state).eval(); }, subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> subtract)
{
  last_update_accepted = false;
  if (x.size() == 0 || z.size() == 0 || H.cols() != x.size() || H.rows() != z.size() ||
      R.rows() != z.size() || !covariance_ok(R) || std::isnan(nis_gate) || nis_gate <= 0.0)
    throw std::invalid_argument("EKF observation dimension, covariance or gate is invalid");
  const Eigen::VectorXd expected = h(x);
  if (expected.size() != z.size()) throw std::invalid_argument("EKF observation function dimension");
  if (!z.allFinite() || !expected.allFinite() || !H.allFinite()) {
    log_innovation(Eigen::VectorXd::Zero(z.size()), std::numeric_limits<double>::infinity(), false);
    return x;
  }
  const Eigen::VectorXd innovation = subtract(z, expected);
  if (innovation.size() != z.size()) throw std::invalid_argument("EKF residual dimension");
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  S = 0.5 * (S + S.transpose()).eval();
  Eigen::LDLT<Eigen::MatrixXd> factor(S);
  if (!innovation.allFinite() || !S.allFinite() || factor.info() != Eigen::Success ||
      factor.vectorD().minCoeff() <= 1e-12) {
    log_innovation(innovation, std::numeric_limits<double>::infinity(), false);
    return x;
  }
  const double nis = innovation.dot(factor.solve(innovation));
  if (!std::isfinite(nis) || nis < -1e-9 || nis > nis_gate) {
    log_innovation(innovation, nis, false);
    return x;
  }
  const Eigen::MatrixXd K = factor.solve((P * H.transpose()).transpose()).transpose();
  const Eigen::VectorXd corrected_x = x_add(x, K * innovation);
  const Eigen::MatrixXd correction = I - K * H;
  const Eigen::MatrixXd corrected_P = correction * P * correction.transpose() + K * R * K.transpose();
  if (corrected_x.size() != x.size() || !corrected_x.allFinite() || !corrected_P.allFinite()) {
    log_innovation(innovation, nis, false);
    return x;
  }
  x = corrected_x;
  P = 0.5 * (corrected_P + corrected_P.transpose()).eval();
  last_update_accepted = true;
  log_innovation(innovation, nis, true);
  return x;
}

void ExtendedKalmanFilter::log_innovation(
  const Eigen::VectorXd & residual, double nis, bool accepted)
{
  last_nis = nis;
  recent_nis_failures.push_back(accepted ? 0 : 1);
  while (recent_nis_failures.size() > std::max<std::size_t>(window_size, 1))
    recent_nis_failures.pop_front();
  data["nis"] = nis;
  data["nis_fail"] = accepted ? 0.0 : 1.0;
  data["measurement_dimension"] = static_cast<double>(residual.size());
  data["recent_nis_failures"] = static_cast<double>(
    std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0)) /
    static_cast<double>(recent_nis_failures.size());
  for (auto it = data.begin(); it != data.end();) {
    if (it->first.rfind("residual_", 0) == 0) it = data.erase(it);
    else ++it;
  }
  for (Eigen::Index i = 0; i < residual.size(); ++i)
    data["residual_" + std::to_string(i)] = residual[i];
  if (residual.size() == 4) {
    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
  }
}
}  // namespace tools
