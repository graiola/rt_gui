#ifndef RT_GUI_SUPPORT_COMMON_H
#define RT_GUI_SUPPORT_COMMON_H

#include <ros/ros.h>

#include <rt_gui/Bool.h>
#include <rt_gui/Double.h>
#include <rt_gui/Int.h>
#include <rt_gui/List.h>
#include <rt_gui/Void.h>
#include <rt_gui/Text.h>

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <tuple>

namespace rt_gui
{


typedef std::pair<std::string,std::string> key_t;

template <class data_t>
class CallbackBuffer {

public:

  using callback_t = std::function<void(data_t)>;

  CallbackBuffer() {}
  //                   0       1         2       3   4 - 0 client data, 1 buffer data, 2 callback, 3 sync, 4 read only data
  typedef std::tuple<data_t*,data_t,callback_t,bool,bool> value_t;
  typedef std::map<key_t,value_t> buffer_t;

  bool add(const std::string& key1, const std::string& key2, data_t data, callback_t callback, bool sync = true)
  {
    if(callback!=nullptr)
    {
      buffer_[key_t(key1,key2)] = value_t(nullptr,data,callback,sync,false);
      old_values_[key_t(key1,key2)] = data;
      return true;
    }
    else
      return false;
  }

  bool add(const std::string& key1, const std::string& key2, data_t* data_ptr, bool sync = true, bool read_only = false)
  {
    if(data_ptr!=nullptr)
    {
      buffer_[key_t(key1,key2)] = value_t(data_ptr,*data_ptr,nullptr,sync,read_only);
      old_values_[key_t(key1,key2)] = *data_ptr;
      return true;
    }
    else
      return false;
  }

  data_t update(const std::string& key1, const std::string& key2, const data_t& value)
  {
    data_t actual_value;
    if (std::get<0>(buffer_[key_t(key1,key2)])!=nullptr) // get the actual value from the buffer before writing it with the new one
      actual_value = *std::get<0>(buffer_[key_t(key1,key2)]);
    else
      actual_value = value;

    if(!std::get<4>(buffer_[key_t(key1,key2)])) // read_only - If not read only, copy the data from buffer to pointer
    {
      if(std::get<3>(buffer_[key_t(key1,key2)])) // sync - copy the new data in the buffer
        std::get<1>(buffer_[key_t(key1,key2)]) = value;
      else // copy the data directly into the raw data or call the callback
      {
        if (std::get<0>(buffer_[key_t(key1,key2)])!=nullptr) // data pointer still exists
          *std::get<0>(buffer_[key_t(key1,key2)]) = value;
        else if(std::get<2>(buffer_[key_t(key1,key2)])!=nullptr) // callback still exists
          std::get<2>(buffer_[key_t(key1,key2)])(value);
        else
          throw std::runtime_error("Missing pointer in buffer!");
      }
    }
    //old_values_[key_t(key1,key2)] = actual_value;
    return actual_value;
  }

  bool sync()
  {
    for(auto tmp_map : buffer_)
    {
      if(std::get<3>(tmp_map.second)) // Sync
      {
        if (std::get<0>(tmp_map.second)!=nullptr) // data pointer still exists
        {
          //old_values_[key_t(tmp_map.first.first,tmp_map.first.second)] = *std::get<0>(tmp_map.second);
          *std::get<0>(tmp_map.second) = std::get<1>(tmp_map.second);
        }
        else if (std::get<2>(tmp_map.second)!=nullptr) // callback still exists
          std::get<2>(tmp_map.second)(std::get<1>(tmp_map.second));
        else
          return false;

      }
    }
    return true;
  }

  bool getValue(const std::string& key1, const std::string& key2, data_t& value)
  {
    if(std::get<0>(buffer_[key_t(key1,key2)])!=nullptr)
    {
      value = *std::get<0>(buffer_[key_t(key1,key2)]);
      return true;
    }
    else
      return false;
  }

  bool isDataChanged(const std::string& key1, const std::string& key2)
  {
    bool changed = false;
    if(std::get<0>(buffer_[key_t(key1,key2)])!=nullptr)
      if(old_values_[key_t(key1,key2)] != *std::get<0>(buffer_[key_t(key1,key2)]))
      {
        changed = true;
        old_values_[key_t(key1,key2)] = *std::get<0>(buffer_[key_t(key1,key2)]);
      }
    return changed;
  }

  const buffer_t& getBuffer()
  {
    return buffer_;
  }

private:
  buffer_t buffer_;
  std::map<key_t,data_t> old_values_;
};

#define RT_GUI_SERVER_NAME "rt_gui_server"
#define RT_GUI_CLIENT_NAME "rt_gui_client"

struct
{
  struct
  {
    std::string add      = "add_double";
    std::string update   = "update_double";
    std::string feedback = "feedback_double";
  } double_srvs;

  struct
  {
    std::string add      = "add_int";
    std::string update   = "update_int";
    std::string feedback = "feedback_int";
  } int_srvs;

  struct
  {
    std::string add      = "add_bool";
    std::string update   = "update_bool";
    std::string feedback = "feedback_bool";
  } bool_srvs;

  struct
  {
    std::string add      = "add_list";
    std::string update   = "update_list";
    std::string feedback = "feedback_list";
  } list_srvs;

  struct
  {
    std::string add      = "add_trigger";
    std::string update   = "update_trigger";
    std::string feedback = "feedback_trigger";
  } trigger_srvs;

  struct
  {
    std::string add      = "add_text";
    std::string update   = "update_text";
    std::string feedback = "feedback_text";
  } text_srvs;

  struct
  {
    std::string add      = "add_label";
    std::string update   = "update_label";
    std::string feedback = "feedback_label";
  } label_srvs;

  unsigned int n_threads = 3;
  double wait_service_secs = 10.0;
  std::string remove_service = "remove_widget";
  std::string add_client = "add_client";

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
    ros::init(argc, argv, "rt_gui_ros_node" ,ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
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
