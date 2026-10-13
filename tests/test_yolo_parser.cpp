// Synthetic network outputs exercise the production parser, not network accuracy.
// No camera, serial port, target truth feed to Controller, or firing is used here.
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

cv::Mat candidate(const std::vector<cv::Point2f> & points, int color, int number)
{
  cv::Mat output(1, 22, CV_32FC1, cv::Scalar(-10.0f));
  // Parser converts network TL,BL,BR,TR into canonical TL,TR,BR,BL.
  const int network_order[] = {0, 3, 2, 1};
  for (int i = 0; i < 4; ++i) {
    output.at<float>(0, i * 2) = points[network_order[i]].x;
    output.at<float>(0, i * 2 + 1) = points[network_order[i]].y;
  }
  output.at<float>(0, 8) = 10.0f;
  output.at<float>(0, 9 + color) = 10.0f;
  output.at<float>(0, 13 + number) = 10.0f;
  return output;
}
}  // namespace

int main(int argc, char * argv[])
{
  try {
    if (argc != 2) throw std::invalid_argument("Usage: yolo_parser_tests configs/offline_cpu.yaml");
    auto_aim::YOLO detector(argv[1], false);
    auto_aim::Solver solver(argv[1]);
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    const cv::Mat canvas(1080, 1440, CV_8UC3, cv::Scalar(0, 0, 0));
    const Eigen::Vector3d position(3.0, 0.0, 0.2);
    const auto points = solver.reproject_armor(position, 0.0, auto_aim::special, auto_aim::outpost);
    require(points.size() == 4, "projection did not produce four corners");

    auto output = candidate(points, 1, 6);
    auto armors = detector.postprocess(1.0, output, canvas, 0);
    require(armors.size() == 1, "production parser rejected the outpost class");
    auto armor = armors.front();
    require(armor.name == auto_aim::outpost && armor.type == auto_aim::special && armor.color == auto_aim::red,
            "outpost number/color/type mapping is incorrect");
    for (std::size_t i = 0; i < 4; ++i)
      require(cv::norm(armor.points[i] - points[i]) < 1e-5, "production corner order is incorrect");
    require(solver.solve(armor), "production outpost parser -> special PnP chain failed");
    require(armor.pose_valid && (armor.xyz_in_world - position).norm() < 0.03,
            "special outpost PnP recovered the wrong metric position");
    std::cout << "PASS synthetic outpost head -> real parser -> special geometry -> PnP\n";

    for (const int number : {7, 8}) {
      output = candidate(points, 0, number);
      armors = detector.postprocess(1.0, output, canvas, 0);
      require(armors.size() == 1 && armors.front().name == auto_aim::base &&
              armors.front().type == auto_aim::special,
              "base-pattern number was discarded or assigned an invented size");
    }
    output = candidate(points, 1, 1);
    armors = detector.postprocess(1.0, output, canvas, 0);
    require(armors.size() == 1 && armors.front().name == auto_aim::one && armors.front().type == auto_aim::big,
            "hero mapping regressed");
    output = candidate(points, 0, 4);
    armors = detector.postprocess(1.0, output, canvas, 0);
    require(armors.size() == 1 && armors.front().name == auto_aim::four && armors.front().type == auto_aim::small,
            "infantry mapping regressed");
    std::cout << "PASS base-pattern 7/8, hero and infantry class contracts\n";

    output.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN();
    require(detector.postprocess(1.0, output, canvas, 0).empty(), "NaN network keypoint was accepted");
    output = candidate(points, 1, 6);
    output.at<float>(0, 0) = output.at<float>(0, 2);
    output.at<float>(0, 1) = output.at<float>(0, 3);
    require(detector.postprocess(1.0, output, canvas, 0).empty(), "degenerate candidate was accepted");
    bool invalid_size_rejected = false;
    try {
      auto_aim::Armor invalid(1, 6, 1.0f, cv::Rect(0, 0, 10, 10), {{0, 0}, {1, 0}, {1, 1}});
    } catch (const std::invalid_argument &) { invalid_size_rejected = true; }
    require(invalid_size_rejected, "three-keypoint constructor did not reject before indexing");
    bool invalid_scale_rejected = false;
    try {
      output = candidate(points, 1, 6);
      detector.postprocess(0.0, output, canvas, 0);
    } catch (const std::invalid_argument &) { invalid_scale_rejected = true; }
    require(invalid_scale_rejected, "invalid rescale factor was accepted");
    std::cout << "PASS NaN, degenerate corners, short constructor input and invalid scale rejection\n";
    std::cout << "All parser tests passed. These are contract tests, not detector accuracy measurements.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL production YOLO parser: " << error.what() << '\n';
    return 1;
  }
}
