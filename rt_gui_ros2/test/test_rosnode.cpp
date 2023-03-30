#include <rt_gui_ros2/support/ros_node.h>

int main(int argc, char **argv)
{
  //rclcpp::init(argc, argv);

  rt_gui::RosNode node("test_rosnode",3);

  rt_gui_msgs::srv::Void srv;

  srv::Request.group_name = "pippo";

 //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
 //  node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
 //
 //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  //rclcpp::spin(node.getNodePtr());
  //rclcpp::shutdown();
}
