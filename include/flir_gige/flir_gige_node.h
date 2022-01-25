#ifndef FLIR_GIGE_NODE_H_
#define FLIR_GIGE_NODE_H_

#include "flir_gige/flir_gige_ros.h"
#include "flir_gige/FlirGigeDynConfig.h"
#include "camera_base/camera_node_base.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace flir_gige {

class FlirGigeNode : public camera_base::CameraNodeBase<FlirGigeDynConfig> {
 public:
  FlirGigeNode()
      : CameraNodeBase(), flir_gige_ros_(*this) {

      this->timer_ = this->create_wall_timer(100ms, std::bind(&FlirGigeNode::spin, this));
    }

  virtual void Acquire() override;
  virtual void Setup(FlirGigeDynConfig &config) override;
  void spin();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  FlirGigeRos flir_gige_ros_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_NODE_H_
