#include "flir_gige/flir_gige_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<flir_gige::FlirGigeNode> main_node = std::make_shared<flir_gige::FlirGigeNode>();
  try {

    main_node->Init();

    rclcpp::spin(main_node);
    rclcpp::shutdown();
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(main_node->get_logger(), e.what());
  }
}
