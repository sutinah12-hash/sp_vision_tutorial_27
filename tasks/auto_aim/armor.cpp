#include "armor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace auto_aim
{
namespace
{
void set_geometry(Armor & armor)
{
  if (armor.points.size() != 4 || !std::isfinite(armor.confidence))
    throw std::invalid_argument("Armor requires four finite keypoints and finite confidence");
  for (const auto & point : armor.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        std::abs(point.x) > 1e6f || std::abs(point.y) > 1e6f)
      throw std::invalid_argument("Armor keypoint is non-finite or out of bounds");
  }
  const auto & p = armor.points;
  const double left_height = cv::norm(p[0] - p[3]);
  const double right_height = cv::norm(p[1] - p[2]);
  const double top_width = cv::norm(p[0] - p[1]);
  const double bottom_width = cv::norm(p[3] - p[2]);
  if (std::min({left_height, right_height, top_width, bottom_width}) < 1e-6)
    throw std::invalid_argument("Armor keypoints have a degenerate edge");
  armor.center = (p[0] + p[1] + p[2] + p[3]) / 4;
  armor.ratio = std::max(top_width, bottom_width) / std::max(left_height, right_height);
  armor.side_ratio = std::max(left_height, right_height) / std::min(left_height, right_height);
  const auto across = (p[1] + p[2] - p[0] - p[3]) / 2;
  const double roll = std::atan2(across.y, across.x);
  const double left_error = std::abs(std::atan2((p[3] - p[0]).y, (p[3] - p[0]).x) - roll - CV_PI / 2);
  const double right_error = std::abs(std::atan2((p[2] - p[1]).y, (p[2] - p[1]).x) - roll - CV_PI / 2);
  armor.rectangular_error = std::max(left_error, right_error);
}

std::vector<cv::Point2f> shifted(std::vector<cv::Point2f> points, const cv::Point2f & offset)
{
  if (!std::isfinite(offset.x) || !std::isfinite(offset.y) ||
      std::abs(offset.x) > 1e6 || std::abs(offset.y) > 1e6)
    throw std::invalid_argument("Non-finite or out-of-bounds ROI offset");
  for (auto & point : points) point += offset;
  return points;
}

void set_yolov5_class(Armor & armor, int color_id, int num_id)
{
  // Color head has four scores, matching the supplied red/blue/off/purple convention.
  static const Color colors[] = {blue, red, extinguish, purple};
  armor.color = color_id >= 0 && color_id < 4 ? colors[color_id] : unknown;
  if (armor.color == unknown) return;
  switch (num_id) {
    case 0: armor.name = sentry; armor.type = small; break;
    case 1: armor.name = one; armor.type = big; break;
    case 2: armor.name = two; armor.type = small; break;
    case 3: armor.name = three; armor.type = small; break;
    case 4: armor.name = four; armor.type = small; break;
    case 5: armor.name = five; armor.type = small; break;
    case 6: armor.name = outpost; armor.type = special; break;
    // The repository's property table has two base-pattern groups, both special.
    // The XML contains no fine class labels: do not invent a large/small distinction.
    // Neither base-pattern class is a task-2/3 firing target.
    case 7:
    case 8: armor.name = base; armor.type = special; break;
    default: armor.name = not_armor; armor.color = unknown; break;
  }
}
}  // namespace

Lightbar::Lightbar(const cv::RotatedRect & rect, std::size_t index)
: id(index), rotated_rect(rect)
{
  if (!std::isfinite(rect.center.x) || !std::isfinite(rect.center.y) ||
      !std::isfinite(rect.size.width) || !std::isfinite(rect.size.height) ||
      !std::isfinite(rect.angle) || rect.size.width <= 0 || rect.size.height <= 0)
    throw std::invalid_argument("Invalid lightbar rectangle");
  std::vector<cv::Point2f> corners(4);
  rect.points(corners.data());
  std::sort(corners.begin(), corners.end(), [](const auto & a, const auto & b) { return a.y < b.y; });
  center = rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;
  points = {top, bottom};
  width = cv::norm(corners[0] - corners[1]);
  length = cv::norm(top2bottom);
  if (width <= 1e-6 || length <= 1e-6) throw std::invalid_argument("Degenerate lightbar");
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  ratio = length / width;
}

Armor::Armor(const Lightbar & left_bar, const Lightbar & right_bar)
: color(left_bar.color), left(left_bar), right(right_bar),
  points{left_bar.top, right_bar.top, right_bar.bottom, left_bar.bottom}
{
  set_geometry(*this);
}

Armor::Armor(int id, float score, const cv::Rect & rectangle, std::vector<cv::Point2f> keypoints)
: points(std::move(keypoints)), class_id(id), box(rectangle), confidence(score)
{
  set_geometry(*this);
  if (id >= 0 && static_cast<std::size_t>(id) < armor_properties.size()) {
    std::tie(color, name, type) = armor_properties[static_cast<std::size_t>(id)];
  }
}

Armor::Armor(int id, float score, const cv::Rect & rectangle,
             std::vector<cv::Point2f> keypoints, cv::Point2f offset)
: Armor(id, score, rectangle, shifted(std::move(keypoints), offset))
{
  box.x += cvRound(offset.x);
  box.y += cvRound(offset.y);
}

Armor::Armor(int color_id, int num_id, float score, const cv::Rect & rectangle,
             std::vector<cv::Point2f> keypoints)
: points(std::move(keypoints)), box(rectangle), confidence(score)
{
  set_geometry(*this);
  set_yolov5_class(*this, color_id, num_id);
}

Armor::Armor(int color_id, int num_id, float score, const cv::Rect & rectangle,
             std::vector<cv::Point2f> keypoints, cv::Point2f offset)
: Armor(color_id, num_id, score, rectangle, shifted(std::move(keypoints), offset))
{
  box.x += cvRound(offset.x);
  box.y += cvRound(offset.y);
}
}  // namespace auto_aim
