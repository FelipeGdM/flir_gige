#include "flir_gige/flir_gige_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  FlirGigeDynConfig flir_config = {
      .raw = false,
      .nuc_action = false,
      .nuc_mode = 0,
      .fps = 20,
  };

  std::shared_ptr<flir_gige::FlirGigeNode> main_node =
      std::make_shared<flir_gige::FlirGigeNode>();
  try {
    main_node->Init();

    main_node->ConfigCb(flir_config, 0);

    rclcpp::spin(main_node);
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(main_node->get_logger(), e.what());
  }
}
