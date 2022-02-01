#ifndef FLIR_GIGE_NODE_H_
#define FLIR_GIGE_NODE_H_

#include "camera_base/camera_node_base.h"
#include "flir_gige/FlirGigeDynConfig.h"
#include "flir_gige/flir_gige_ros.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace flir_gige {

class FlirGigeNode : public camera_base::CameraNodeBase<FlirGigeDynConfig> {
 public:
  FlirGigeNode()
      : CameraNodeBase(),
        timer_(this->create_wall_timer(500ms,
                                       std::bind(&FlirGigeNode::spin, this))){};

  void Init();

  virtual void Acquire() override;
  virtual void Setup(FlirGigeDynConfig &config) override;
  void spin();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<FlirGigeRos> flir_gige_ros_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_NODE_H_
