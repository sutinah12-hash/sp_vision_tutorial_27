#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"

#include <chrono>
#include <string>

int main()
{
  io::Camera camera("./configs/camera.yaml");
  auto_charge::AprilTagDetector detector("./configs/yolo.yaml");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  while (true) {
    camera.read(img, timestamp);
    if (img.empty()) continue;

    const auto detections = detector.detect(img);
    for (const auto & detection : detections) {
      if (detection.corners.size() != 4) continue;

      for (size_t i = 0; i < detection.corners.size(); ++i) {
        const auto & current = detection.corners[i];
        const auto & next = detection.corners[(i + 1) % detection.corners.size()];
        cv::line(img, current, next, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      }

      cv::circle(img, detection.center, 5, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
      cv::putText(
        img, "ID " + std::to_string(detection.id), detection.corners[0] + cv::Point2f(0, -10),
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow("apriltag", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
