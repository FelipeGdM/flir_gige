#pragma once

#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <type_traits>

namespace camera_base {

/**
 * @brief getParam Util function for getting ros parameters under nodehandle
 * @param nh Node handle
 * @param name Parameter name
 * @return Parameter value
 */
inline rclcpp::Parameter getParam(rclcpp::Node::SharedPtr node, const std::string& name) {
  rclcpp::Parameter value{};
  if (!node->get_parameter(name, value)) {
    RCLCPP_ERROR(node->get_logger(), "Cannot find parameter: %s", name.c_str());
  }
  return value;
}

/**
 * @brief The CameraRosBase class
 * This class implements a ros camera
 */
class CameraRosBase {
 public:
  explicit CameraRosBase(rclcpp::Node::SharedPtr node,
                         const std::string& prefix = std::string())
      : node_(node),
        it_(node_),
        camera_pub_(it_.advertiseCamera("image_raw", 1)),
        fps_(10.0),
        diagnostic_updater_(node_),
        topic_diagnostic_(
            prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
            diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {

    node_->declare_parameter<std::string>("camera_name", "default_camera_name");
    node_->declare_parameter<std::string>("calib_url", "default_calib_url");

    cinfo_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      node_.get(),
      getParam(node_, "camera_name").value_to_string(),
      getParam(node_, "calib_url").value_to_string());

    rclcpp::Parameter frame_id_par_, identifier_par_;

    // FIXME Hardcoded frame_id
    node_->get_parameter_or("frame_id", frame_id_par_, rclcpp::Parameter("frame_id", "thermal"));

    // FIXME Hardcoded camera ip address
    node_->get_parameter_or("identifier", identifier_par_, rclcpp::Parameter("identifier", "172.16.20.238"));

    frame_id_ = frame_id_par_.value_to_string();
    identifier_ = identifier_par_.value_to_string();
  }

  CameraRosBase() = delete;
  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }
  const std::string& frame_id() const { return frame_id_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }

  /**
   * @brief SetHardwareId Set hardware id for diagnostic updater
   * @param id harware id
   */
  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  /**
   * @brief PublishCamera Publish a camera topic with Image and CameraInfo
   * @param time Acquisition time stamp
   */
  void PublishCamera(const rclcpp::Time& time) {
    const auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    const auto cinfo_msg =
        std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_mgr_->getCameraInfo());
    image_msg->header.frame_id = frame_id_;
    image_msg->header.stamp = time;
    if (Grab(image_msg, cinfo_msg)) {
      // Update camera info header
      cinfo_msg->header = image_msg->header;
      camera_pub_.publish(image_msg, cinfo_msg);
      topic_diagnostic_.tick(image_msg->header.stamp);
    }
    // TODO! Should this be done?
    // diagnostic_updater_.update();
    diagnostic_updater_.force_update();
  }

  void Publish(const sensor_msgs::msg::Image::Ptr& image_msg) {
    const auto cinfo_msg =
        std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_mgr_->getCameraInfo());
    // Update camera info header
    image_msg->header.frame_id = frame_id_;
    cinfo_msg->header = image_msg->header;
    camera_pub_.publish(image_msg, cinfo_msg);
    topic_diagnostic_.tick(image_msg->header.stamp);
    // TODO! Should this be done?
    // diagnostic_updater_.update();
    diagnostic_updater_.force_update();
  }

  /**
   * @brief Grab Fill image_msg and cinfo_msg from low level camera driver
   * @param image_msg Ros message ImagePtr
   * @return True if successful
   */
  virtual bool Grab(const sensor_msgs::msg::Image::Ptr& image_msg,
                    const sensor_msgs::msg::CameraInfo::Ptr& cinfo_msgs = nullptr) = 0;

 private:
  rclcpp::Node::SharedPtr node_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;
  double fps_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;

  std::string frame_id_;
  std::string identifier_;
};

}  // namespace camera_base
