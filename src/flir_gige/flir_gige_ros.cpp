#include "flir_gige/flir_gige_ros.h"

namespace flir_gige {

bool FlirGigeRos::Grab(const sensor_msgs::msg::Image::Ptr& image_msg,
                       const sensor_msgs::msg::CameraInfo::Ptr& cinfo_msg) {
  return flir_gige_.GrabImage(*image_msg, *cinfo_msg);
}

void FlirGigeRos::PublishTemperature(const rclcpp::Time& time) {
  if (flir_gige_.GrabTemprature(temp_msg_)) {
    temp_msg_.header.stamp = time;
    temp_msg_.header.frame_id = frame_id();
    temp_pub_->publish(temp_msg_);
  }
}

}  // namespace flir_gige
