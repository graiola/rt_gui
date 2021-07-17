#ifndef SUPPORT_COMMON_H
#define SUPPORT_COMMON_H

#include <ros/ros.h>

#include <rt_gui/addCheckBox.h>
#include <rt_gui/addComboBox.h>
#include <rt_gui/addRadioButton.h>
#include <rt_gui/addSlider.h>

#include <rt_gui/updateCheckBox.h>
#include <rt_gui/updateComboBox.h>
#include <rt_gui/updateRadioButton.h>
#include <rt_gui/updateSlider.h>

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

namespace rt_gui
{

#define RT_GUI_SERVER_NAME "rt_gui_server"
#define RT_GUI_CLIENT_NAME "rt_gui_client"

struct
{
  struct
  {
    std::string add    = "add_slider";
    std::string update = "update_slider";
  } slider;

  struct
  {
    std::string add    = "add_radio_button";
    std::string update = "update_radio_button";
  } radio_button;

  struct
  {
    std::string add    = "add_combo_box";
    std::string update = "update_combo_box";
  } combo_box;

  unsigned int n_threads = 3;

} _ros_services;

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
    ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
    free(arg0);

    if(ros::master::check())
    {
      ros_nh_.reset(new ros::NodeHandle(ros_node_name));
    }
    else
    {
      throw std::runtime_error("roscore not found... Did you start the server?");
    }

    spinner_.reset(new ros::AsyncSpinner(n_threads)); // Use one thread to keep the ros magic alive
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
