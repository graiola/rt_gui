#ifndef RT_GUI_COMMON_H
#define RT_GUI_COMMON_H

#include <ros/ros.h>

#include <memory>
#include <atomic>
#include <thread>

namespace rt_gui
{

#define RT_GUI_SERVER_NAME "rt_gui_server"
#define RT_GUI_CLIENT_NAME "rt_gui_client"

class RosNode
{
public:
  RosNode(const std::string& ros_node_name)
  {
    init(ros_node_name);
  }

  RosNode()
  {
    init_ = false;
  }

  void init(const std::string& ros_node_name)
  {
    int argc = 1;
    char* arg0 = strdup(ros_node_name.c_str());
    char* argv[] = {arg0, nullptr};
    ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
    free(arg0);

    if(ros::master::check())
    {
      ros_nh_.reset(new ros::NodeHandle(ros_node_name));
    }
    else
    {
      std::string err("roscore not found... Did you start the server?");
      throw std::runtime_error(err);
    }

    spinner_.reset(new ros::AsyncSpinner(1)); // Use one thread to keep the ros magic alive
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
    {
      std::string err("RosNode not initialized");
      throw std::runtime_error(err);
    }
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
      std::string err("RosNode not initialized");
      throw std::runtime_error(err);
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
