#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <string>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // NIS uses the prior innovation. NEES requires truth and is intentionally not logged.
  std::map<std::string, double> data;
  std::deque<int> recent_nis_failures;
  size_t window_size = 100;
  double last_nis = 0.0;
  double nis_gate = std::numeric_limits<double>::infinity();
  bool last_update_accepted = false;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
    [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return (a + b).eval(); };
  void log_innovation(const Eigen::VectorXd & residual, double nis, bool accepted);
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
