#ifndef RT_GUI_COMMON_H
#define RT_GUI_COMMON_H

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

typedef std::pair<std::string,std::string> buffer_key_t;

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

class SliderClientManager
{

public:

  typedef std::pair<double*,double> buffer_value_t;
  typedef std::map<buffer_key_t, buffer_value_t> buffer_t;

  typedef std::shared_ptr<SliderClientManager> Ptr;

  SliderClientManager(ros::NodeHandle& node)
  {
     add_ = node.serviceClient<rt_gui::addSlider>("/" RT_GUI_SERVER_NAME "/add_slider");
     update_ = node.advertiseService("update_slider", &SliderClientManager::update, this);
  }

  bool update(updateSlider::Request &req,
              updateSlider::Response &res)
  {
     sync_mtx_.lock();
     buffer_[buffer_key_t(req.group_name,req.data_name)].second = req.value;
     sync_mtx_.unlock();
     // FIXME add a proper error handling
     res.resp = true;

     return res.resp;
  }

  void add(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
  {
    assert(data_ptr);
    rt_gui::addSlider srv;
    srv.request.min = min;
    srv.request.max = max;
    srv.request.init = *data_ptr;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    if(add_.exists())
    {
      add_.call(srv);
      if(srv.response.resp == false)
        throw std::runtime_error("RtGuiServer::addSlider::resp is false!");
      else
        buffer_[buffer_key_t(group_name,data_name)] = buffer_value_t(data_ptr,*data_ptr);
    }
    else
    {
      throw std::runtime_error("RtGuiServer::addSlider service is not available!");
    }
  }

  bool sync()
  {
    if(sync_mtx_.try_lock())
    {
      for(auto tmp_map : buffer_)
        if(tmp_map.second.first!=nullptr) // The data pointer still exists
          *tmp_map.second.first = tmp_map.second.second;
      sync_mtx_.unlock();
    }
  }

private:
  ros::ServiceServer update_;
  ros::ServiceClient add_;
  buffer_t buffer_;
  std::mutex sync_mtx_;

};

} // namespace


#endif
