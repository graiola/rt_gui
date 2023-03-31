#include <rt_gui_ros2/support/ros_node.h>
#include <unistd.h>

int main(int argc, char **argv)
{

  rt_gui::RosNode node("test_rosnode",3);

 //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
 //  node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
 //
 //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  while(rclcpp::ok())
  {
    std::cout << "Test..." << std::endl;
    usleep(3000000);
  }

  return 0;
}
