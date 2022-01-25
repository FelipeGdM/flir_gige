#include "flir_gige/flir_gige_node.h"

namespace flir_gige {

void FlirGigeNode::Setup(FlirGigeDynConfig &config) {
  flir_gige_ros_.set_fps(config.fps);
  flir_gige_ros_.Reconnect();
  flir_gige_ros_.camera().Configure(config);
  flir_gige_ros_.Start();
}

void FlirGigeNode::Acquire() {
  while (is_acquire() && rclcpp::ok()) {
    const rclcpp::Time time = this->now();
    flir_gige_ros_.PublishCamera(time);
    flir_gige_ros_.PublishTemperature(time);
    Sleep();
  }
}

void FlirGigeNode::spin() {
  // Do stuff
}

}  // namespace flir_gige
