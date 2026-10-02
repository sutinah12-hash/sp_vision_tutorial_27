#pragma once
#include "tracking_controller.hpp"
namespace pid_controller {
// PID + velocity feedforward baseline; all three interfaces are implemented by TrackingController.
class PidController : public nav_tracking::TrackingController {
 public:
  PidController() : TrackingController(false) {}
};
}
