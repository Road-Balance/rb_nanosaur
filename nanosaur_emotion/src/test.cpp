
#include "nanosaur_emotion/test.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_loop_node");
  auto me = MyInfo();

  // WallRate & Rate
  // https://qiita.com/NeK/items/bcf518f6dd79f970bb8e
  rclcpp::WallRate rate(2);

  while (rclcpp::ok()) {
    // RCLCPP_INFO(node->get_logger(), "Simple Loop Node");
    me.get_info();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
