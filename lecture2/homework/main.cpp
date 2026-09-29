#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
  io::Camera camera("./configs/camera.yaml");
  auto_aim::YOLO yolo("./configs/yolo.yaml");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  while (true) {
    camera.read(img, timestamp);
    if (img.empty()) continue;

    const auto armors = yolo.detect(img);
    for (const auto & armor : armors) {
      if (armor.points.size() == 4) {
        tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
        const auto label =
          auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];
        const cv::Point label_origin(armor.box.x, armor.box.y - 10);
        tools::draw_text(img, label, label_origin, cv::Scalar(0, 255, 0), 0.8, 2);
      }
    }

    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
