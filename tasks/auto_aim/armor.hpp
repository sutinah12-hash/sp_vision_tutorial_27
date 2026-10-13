#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple,
  unknown
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple", "unknown"};

enum ArmorType
{
  big,
  small,
  special
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small", "special"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, big},        {red, one, big},        {extinguish, one, big},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, special},    {red, outpost, special},    {extinguish, outpost, special},
  {blue, base, special},         {red, base, special},         {extinguish, base, special},      {purple, base, special},       
  {blue, base, special},       {red, base, special},       {extinguish, base, special},    {purple, base, special},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  std::size_t id = 0;
  Color color = unknown;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle = 0.0, angle_error = 0.0, length = 0.0, width = 0.0, ratio = 0.0;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor
{
  Color color = unknown;
  Lightbar left, right;     //used to be const
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio = 0.0;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio = 0.0;         // 长灯条与短灯条的长度之比
  double rectangular_error = 0.0;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type = small;
  ArmorName name = not_armor;
  ArmorPriority priority = fifth;
  int class_id = -1;
  cv::Rect box;
  cv::Mat pattern;
  double confidence = 0.0;
  bool duplicated = false;

  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系

  double yaw_raw = 0.0;  // rad
  double reprojection_error = 0.0;  // px, populated by Solver
  bool pose_valid = false;
  double raw_pitch = 0.0;  // rad, before any geometry-constrained refinement

  Armor() = default;
  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
