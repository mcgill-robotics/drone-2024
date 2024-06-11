#include <cstdio>
#include <gui_drone/gui_node.hpp>
#include <rclcpp/qos.hpp>

GuiNode::GuiNode(): rclcpp::Node("gui_node") {

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1));
  qos.best_effort();
  qos.transient_local();
  start_publisher = this->create_publisher<std_msgs::msg::String>("/start", qos);

}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world gui_drone package\n");
  return 0;
}
