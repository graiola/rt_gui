#ifndef RT_GUI_ROS_SUPPORT_ROS_NODE_H
#define RT_GUI_ROS_SUPPORT_ROS_NODE_H

#include <ros/ros.h>

#include <rt_gui_msgs/Bool.h>
#include <rt_gui_msgs/Double.h>
#include <rt_gui_msgs/Int.h>
#include <rt_gui_msgs/List.h>
#include <rt_gui_msgs/Void.h>
#include <rt_gui_msgs/Text.h>

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
    char* arg0 = strdup(ros_node_name.c_str());
    char* argv[] = {arg0, nullptr};
    ros::init(argc, argv, "rt_gui_ros_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    free(arg0);

    // Search for the substring in string
    std::string ros_node_name_fix = ros_node_name;
    std::string arg = "__name:=";
    size_t idx = ros_node_name_fix.find(arg);
    if (idx != std::string::npos)
        ros_node_name_fix.erase(idx, arg.length());

    if(ros::master::check())
    {
      ros_nh_.reset(new ros::NodeHandle(ros_node_name_fix));
    }
    else
    {
      throw std::runtime_error("roscore not found... did you start the server?");
    }

    spinner_.reset(new ros::AsyncSpinner(n_threads)); // Use n_threads to keep the ros magic alive
    spinner_->start();

    init_ = true;
  }

  ~RosNode()
  {
    if(init_ == true)
    {
      ros_nh_->shutdown();
      spinner_->stop();
    }
  }

  ros::NodeHandle& getNode()
  {
    if(init_ == true)
      return *ros_nh_.get();
    else
      throw std::runtime_error("RosNode not initialized");
  }

  std::shared_ptr<ros::NodeHandle> getNodePtr()
  {
    if(init_ == true)
      return ros_nh_;
    else
      throw std::runtime_error("RosNode not initialized");
  }

  bool reset()
  {
    if(init_ == true)
    {
      ros_nh_->shutdown();
      spinner_->stop();
      init_ = false;
      return true;
    }
    else
    {
      throw std::runtime_error("RosNode not initialized");
      return false;
    }
  }

  bool initDone()
  {
    return init_;
  }

protected:
  bool init_;
  std::shared_ptr<ros::NodeHandle> ros_nh_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
};

} // namespace

#endif
