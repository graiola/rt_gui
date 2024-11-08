#ifndef RT_GUI_ROS_SUPPORT_ROS_NODE_H
#define RT_GUI_ROS_SUPPORT_ROS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/init_options.hpp>

#include <rt_gui_msgs/srv/bool.hpp>
#include <rt_gui_msgs/srv/check.hpp>
#include <rt_gui_msgs/srv/double.hpp>
#include <rt_gui_msgs/srv/int.hpp>
#include <rt_gui_msgs/srv/list.hpp>
#include <rt_gui_msgs/srv/void.hpp>
#include <rt_gui_msgs/srv/text.hpp>

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <stdexcept>

namespace rt_gui
{

class RosNode
{
public:
  explicit RosNode(const std::string& ros_node_name, unsigned int n_threads)
    : init_(false)
  {
    init(ros_node_name, n_threads);
  }

  RosNode() : init_(false) {}

  ~RosNode()
  {
    shutdown();
  }

  void init(const std::string& ros_node_name, unsigned int n_threads)
  {
    if (init_) return;  // Skip re-initialization

    int argc = 1;
    char* arg0 = strdup("");
    char* argv[] = {arg0, nullptr};

    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);
      rclcpp::uninstall_signal_handlers();
    }
    free(arg0);

    ros_nh_ = std::make_shared<rclcpp::Node>(ros_node_name);

    spinner_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), n_threads);
    spinner_->add_node(ros_nh_);

    init_ = true;

    spinner_thread_ = std::make_unique<std::thread>(&rclcpp::executors::MultiThreadedExecutor::spin, spinner_.get());
  }

  rclcpp::Node& getNode()
  {
    if (!init_)
      throw std::runtime_error("Error: RosNode not initialized.");
    return *ros_nh_;
  }

  std::shared_ptr<rclcpp::Node> getNodePtr()
  {
    if (!init_)
      throw std::runtime_error("Error: RosNode not initialized.");
    return ros_nh_;
  }

  void reset()
  {
    shutdown();
    init_ = false;
  }

  bool initDone() const
  {
    return init_;
  }

private:
  void shutdown()
  {
    if (init_) {
      if (spinner_thread_ && spinner_thread_->joinable()) {
        spinner_->cancel();
        spinner_thread_->join();
      }
      rclcpp::shutdown();
      init_ = false;
    }
  }

  std::atomic<bool> init_;
  std::shared_ptr<rclcpp::Node> ros_nh_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> spinner_;
  std::unique_ptr<std::thread> spinner_thread_;
};

} // namespace rt_gui

#endif // RT_GUI_ROS_SUPPORT_ROS_NODE_H
