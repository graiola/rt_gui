#ifndef RT_GUI_ROS2_SUPPORT_ROS_NODE_H
#define RT_GUI_ROS2_SUPPORT_ROS_NODE_H

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
#include <tuple>

namespace rt_gui
{

class RosNode
{

public:
  RosNode(const std::string& ros_node_name, const unsigned int& n_threads)
  {
    init(ros_node_name,n_threads);
  }

  RosNode()
  {
    init_ = false;
  }

  void init(const std::string& ros_node_name, const unsigned int& n_threads)
  {
    int argc = 1;
    char* arg0 = strdup("");//strdup(ros_node_name.c_str());
    char* argv[] = {arg0, nullptr};
    rclcpp::init(argc, argv);
    rclcpp::uninstall_signal_handlers();
    free(arg0);

    ros_nh_.reset(new rclcpp::Node(ros_node_name));

    //spinner_.reset(new rclcpp::executors::MultiThreadedExecutor(n_threads));
    spinner_.reset(new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(),n_threads));
    spinner_->add_node(ros_nh_);

    init_ = true;

    spinner_thread_.reset(new std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, spinner_.get())));
    spinner_thread_->detach();
  }

  ~RosNode()
  {
    if(init_ == true)
    {
      spinner_thread_->join();
      rclcpp::shutdown();
    }
  }

  rclcpp::Node& getNode()
  {
    if(init_ == true)
      return *ros_nh_.get();
    else
      throw std::runtime_error("RosNode not initialized");
  }

  std::shared_ptr<rclcpp::Node> getNodePtr()
  {
    if(init_ == true)
      return ros_nh_;
    else
      throw std::runtime_error("RosNode not initialized");
  }

  void reset()
  {
    if(init_ == true)
    {
      spinner_thread_->join();
      rclcpp::shutdown();
      init_ = false;
    }
    else
      throw std::runtime_error("RosNode not initialized");
  }

  bool initDone()
  {
    return init_;
  }

protected:
  bool init_;
  std::shared_ptr<rclcpp::Node> ros_nh_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> spinner_;
  std::shared_ptr<std::thread> spinner_thread_;
};

} // namespace

#endif
