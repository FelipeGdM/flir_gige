#include "flir_gige/flir_gige_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<flir_gige::FlirGigeNode>());
    rclcpp::shutdown();
  }
  catch (const std::exception &e) {
    // ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
