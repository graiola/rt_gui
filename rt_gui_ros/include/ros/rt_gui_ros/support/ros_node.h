#ifndef RT_GUI_ROS_SUPPORT_ROS_NODE_H
#define RT_GUI_ROS_SUPPORT_ROS_NODE_H

#include <ros/ros.h>
#include <rt_gui_msgs/Bool.h>
#include <rt_gui_msgs/Check.h>
#include <rt_gui_msgs/Double.h>
#include <rt_gui_msgs/Int.h>
#include <rt_gui_msgs/List.h>
#include <rt_gui_msgs/Void.h>
#include <rt_gui_msgs/Text.h>

#include <memory>
#include <optional>
#include <thread>
#include <mutex>

namespace rt_gui
{

class RosNode
{
public:
  RosNode(const std::string& ros_node_name, unsigned int n_threads) noexcept(false)
  {
    init(ros_node_name, n_threads);
  }

  RosNode() = default;

  void init(const std::string& ros_node_name, unsigned int n_threads) noexcept(false)
  {
    static constexpr char arg[] = "__name:=";
    if (ros_nh_ || spinner_)
      reset();

    int argc = 1;
    std::vector<char*> argv{strdup(ros_node_name.c_str()), nullptr};  // Manage argv memory safely
    ros::init(argc, argv.data(), "rt_gui_ros_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    free(argv[0]); // Free after init to avoid memory leak

    std::string ros_node_name_fix = ros_node_name;
    if (auto idx = ros_node_name_fix.find(arg); idx != std::string::npos)
      ros_node_name_fix.erase(idx, sizeof(arg) - 1);

    if (ros::master::check()) {
      ros_nh_ = std::make_shared<ros::NodeHandle>(ros_node_name_fix);
      spinner_ = std::make_unique<ros::AsyncSpinner>(n_threads);
      spinner_->start();
    } else {
      throw std::runtime_error("roscore not found... did you start the server?");
    }
  }

  ~RosNode()
  {
    if (ros_nh_ && spinner_) {
      ros_nh_->shutdown();
      spinner_->stop();
    }
  }

  ros::NodeHandle& getNode()
  {
    if (!ros_nh_) throw std::runtime_error("RosNode not initialized");
    return *ros_nh_;
  }

  std::shared_ptr<ros::NodeHandle> getNodePtr() const
  {
    if (!ros_nh_) throw std::runtime_error("RosNode not initialized");
    return ros_nh_;
  }

  bool reset()
  {
    if (ros_nh_ && spinner_) {
      ros_nh_->shutdown();
      spinner_->stop();
      ros_nh_.reset();
      spinner_.reset();
      return true;
    }
    return false;
  }

  bool initDone() const
  {
    return ros_nh_ && spinner_;
  }

  RosNode(RosNode&& other) noexcept = default;
  RosNode& operator=(RosNode&& other) noexcept = default;
  RosNode(const RosNode&) = delete;
  RosNode& operator=(const RosNode&) = delete;

private:
  std::shared_ptr<ros::NodeHandle> ros_nh_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
};

} // namespace rt_gui

#endif
