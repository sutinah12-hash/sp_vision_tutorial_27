#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
class Target
{
public:
  ArmorName name = not_armor;
  ArmorType armor_type = small;
  ArmorPriority priority = fifth;
  bool jumped = false;
  int last_id = 0;  // Internal association index, never a simulator truth label.
  bool isinit = false;

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius = 0.2,
    int armor_num = 4);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  bool update(const Armor & armor);
  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  Eigen::Vector4d predicted_observation(int id) const;
  Eigen::MatrixXd observation_jacobian(int id) const;

  bool diverged() const;
  void update_convergency();
  bool is_converged() const;
  bool convergened();  // Retained spelling for the course-template API.
  bool checkinit();
  int armor_count() const { return armor_num_; }
  int accepted_updates() const { return update_count_; }
  int rejected_updates() const { return rejected_count_; }
  std::chrono::steady_clock::time_point state_time() const { return t_; }
  std::chrono::steady_clock::time_point last_observation_time() const { return last_observation_t_; }

private:
  int armor_num_ = 0;
  int switch_count_ = 0;
  int update_count_ = 0;
  int rejected_count_ = 0;
  int consecutive_rejections_ = 0;
  bool is_converged_ = false;
  bool geometry_enabled_ = false;
  std::vector<int> seen_ids_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_{};
  std::chrono::steady_clock::time_point initial_t_{};
  std::chrono::steady_clock::time_point last_observation_t_{};

  bool update_ypda(const Armor & armor, int id);
  bool is_layered_outpost() const;
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::Vector4d h_observation(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
  Eigen::Matrix4d observation_noise(const Armor & armor) const;
};
}  // namespace auto_aim
#endif  // AUTO_AIM__TARGET_HPP
