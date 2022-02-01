#ifndef FLIR_GIGE_ROS_H_
#define FLIR_GIGE_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "camera_base/camera_ros_base.hpp"
#include "flir_gige/flir_gige.hpp"

namespace flir_gige {

class FlirGigeRos : public camera_base::CameraRosBase {
 public:
  FlirGigeRos(rclcpp::Node::SharedPtr node)
      : CameraRosBase(node),
        flir_gige_(identifier()),
        node_(node),
        temp_pub_(
            node->create_publisher<sensor_msgs::msg::Temperature>("spot", 1)),
        temp_msg_(sensor_msgs::msg::Temperature()) {
    SetHardwareId(flir_gige_.display_id());
  }

  FlirGige& camera() { return flir_gige_; }

  void Reconnect() {
    flir_gige_.StopAcquisition();
    flir_gige_.Disconnect();
    flir_gige_.Connect();
  }
  void Start() { flir_gige_.StartAcquisition(); }

  virtual bool Grab(
      const sensor_msgs::msg::Image::Ptr& image_msg,
      const sensor_msgs::msg::CameraInfo::Ptr& cinfo_msg) override;

  void PublishTemperature(const rclcpp::Time& time);

 private:
  FlirGige flir_gige_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  sensor_msgs::msg::Temperature temp_msg_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_ROS_H_