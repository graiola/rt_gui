#ifndef RT_GUI_ROS2_SUPPORT_CLIENT_H
#define RT_GUI_ROS2_SUPPORT_CLIENT_H

#include <rt_gui_ros2/support/ros_node.h>
#include <rt_gui_core/support/common.h>
#include <type_traits>

namespace rt_gui
{

template<class srv_t, class data_t>
class InterfaceHandler
{

public:

  typedef std::shared_ptr<InterfaceHandler> Ptr;

  typedef std::function<void(data_t)> fun_t;

  InterfaceHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name, bool feedback = false)
  {
    add_srv_       = add_srv;
    update_srv_    = update_srv;
    feedback_srv_  = feedback_srv;
    server_name_   = server_name;
    client_name_   = client_name;
    update_        = node.advertiseService(update_srv, &InterfaceHandler::update, this);
    add_           = node.serviceClient<srv_t>("/"+server_name+"/"+add_srv);
    feedback_      = node.serviceClient<srv_t>("/"+server_name+"/"+feedback_srv);

    stop_feedback_thread_ = false;
    if(feedback)
      feedback_thread_ = std::make_shared<std::thread>(&InterfaceHandler::feedbackLoop,this);

  }

  virtual ~InterfaceHandler() { stop_feedback_thread_ = true; if(feedback_thread_!=nullptr) feedback_thread_->join(); }

  bool update(typename srv_t::Request& req, typename srv_t::Response& res)
  {
    res.resp = updateBuffer(req.group_name,req.data_name,req.value);
    return true;
  }

  bool add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t* data_ptr, bool sync)
  {
    srv_t srv;
    srv.request.value = *data_ptr;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    for(unsigned int i=0;i<list.size();i++)
      srv.request.list.push_back(list[i]);
    return addRawData(group_name,data_name,data_ptr,srv,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t data, fun_t fun, bool sync)
  {
    srv_t srv;
    srv.request.value = data;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    for(unsigned int i=0;i<list.size();i++)
      srv.request.list.push_back(list[i]);
    return addCallback(group_name,data_name,data,fun,srv,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, bool sync, bool read_only = false)
  {
    srv_t srv;
    srv.request.value = *data_ptr;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    return addRawData(group_name,data_name,data_ptr,srv,sync,read_only);
  }

  bool add(const std::string& group_name, const std::string& data_name, data_t data, fun_t fun, bool sync)
  {
    srv_t srv;
    srv.request.value = data;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    return addCallback(group_name,data_name,data,fun,srv,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t* data_ptr, bool sync)
  {
    srv_t srv;
    srv.request.min = min;
    srv.request.max = max;
    srv.request.value = *data_ptr;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    return addRawData(group_name,data_name,data_ptr,srv,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t data, fun_t fun, bool sync)
  {
    srv_t srv;
    srv.request.min = min;
    srv.request.max = max;
    srv.request.value = data;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    return addCallback(group_name,data_name,data,fun,srv,sync);
  }

  bool sync()
  {
    bool res = true;
    if(sync_mtx_.try_lock())
    {
      res = res && buffer_.sync();
      sync_mtx_.unlock();
    }

    return res;
  }

protected:

  bool feedback()
  {
    if(feedback_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
    {
      srv_t srv;
      for(auto tmp_map : buffer_.getBuffer())
      {
        auto group_name = tmp_map.first.first;
        auto data_name = tmp_map.first.second;
        if(buffer_.isDataChanged(group_name,data_name))
        {
          data_t value;
          buffer_.getValue(group_name,data_name,value);
          srv.request.value       = value;
          srv.request.client_name = client_name_;
          srv.request.group_name  = group_name;
          srv.request.data_name   = data_name;
          feedback_.call(srv);
        }
      }
      return true;
    }
    else
      return false;
  }

  void feedbackLoop()
  {
    while(!stop_feedback_thread_)
    {
      sync_mtx_.lock();
      if(!feedback())
      {
         ROS_WARN("RtGuiServer::feedback service is not available!");
         stop_feedback_thread_ = true;
      }
      sync_mtx_.unlock();
      std::this_thread::sleep_for (std::chrono::seconds(1));
    }
  }

  bool addRawData(const std::string& group_name, const std::string& data_name, data_t* data_ptr, srv_t& srv, bool sync, bool read_only = false)
  {
    if(this->add_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
    {
      if(!this->add_.call(srv))
      {
        ROS_WARN("RtGuiServer::add call response is false!");
        return false;
      }
      else
      {
        buffer_.add(group_name,data_name,data_ptr,sync,read_only);
      }
    }
    else
    {
      ROS_WARN("RtGuiServer::add service is not available!");
      return false;
    }
    return true;
  }

  bool addCallback(const std::string& group_name, const std::string& data_name, data_t data, std::function<void(data_t)> fun, srv_t& srv, bool sync)
  {
    if(this->add_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
    {
      if(!this->add_.call(srv))
      {
        ROS_WARN("RtGuiServer::add call response is false!");
        return false;
      }
      else
      {
        buffer_.add(group_name,data_name,data,fun,sync);
      }
    }
    else
    {
      ROS_WARN("RtGuiServer::add service is not available!");
      return false;
    }
    return true;
  }

  data_t updateBuffer(const std::string& group_name, const std::string& data_name, const decltype(srv_t::Request::value)& value)
  {
    sync_mtx_.lock();
    data_t actual_value = buffer_.update(group_name,data_name,value);
    sync_mtx_.unlock();
    return actual_value;
  }

  std::string add_srv_;
  std::string update_srv_;
  std::string feedback_srv_;
  std::string server_name_;
  std::string client_name_;
  srv_t srv_;

  ros::ServiceServer update_;
  ros::ServiceClient feedback_;
  ros::ServiceClient add_;
  CallbackBuffer<data_t> buffer_;
  std::mutex sync_mtx_;

  std::shared_ptr<std::thread> feedback_thread_;
  bool stop_feedback_thread_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerHandler
{

public:

  typedef std::shared_ptr<TriggerHandler> Ptr;

  typedef std::function<void()> fun_t;

  TriggerHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
  {
    add_srv_       = add_srv;
    update_srv_    = update_srv;
    feedback_srv_  = feedback_srv;
    server_name_   = server_name;
    client_name_   = client_name;
    update_        = node.advertiseService(update_srv, &TriggerHandler::update, this);
    add_           = node.serviceClient<rt_gui::Void>("/"+server_name+"/"+add_srv);
    feedback_      = node.serviceClient<rt_gui::Void>("/"+server_name+"/"+feedback_srv);
  }

  bool add(const std::string& group_name, const std::string& data_name, fun_t fun)
  {
    rt_gui::Void srv;
    assert(fun);
    funs_[key_t(group_name,data_name)] = fun;
    srv.request.client_name = client_name_;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    if(this->add_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
    {
      if(!this->add_.call(srv))
      {
        ROS_WARN("RtGuiServer::add call response is false!");
        return false;
      }
    }
    else
    {
      ROS_WARN("RtGuiServer::add service is not available!");
      return false;
    }
    return true;
  }

  bool update(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
  {
    funs_[key_t(req.group_name,req.data_name)]();
    res.resp = true;
    return res.resp;
  }

protected:

  std::string update_srv_;
  std::string add_srv_;
  std::string feedback_srv_;
  std::string server_name_;
  std::string client_name_;

  std::map<key_t,fun_t> funs_;
  ros::ServiceServer update_;
  ros::ServiceClient feedback_;
  ros::ServiceClient add_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntHandler : public InterfaceHandler<rt_gui::Int,int>
{

public:

  typedef std::shared_ptr<IntHandler> Ptr;

  IntHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::Int,int>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleHandler : public InterfaceHandler<rt_gui::Double,double>
{

public:

  typedef std::shared_ptr<DoubleHandler> Ptr;

  DoubleHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::Double,double>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolHandler : public InterfaceHandler<rt_gui::Bool,bool>
{

public:

  typedef std::shared_ptr<BoolHandler> Ptr;

  BoolHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::Bool,bool>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListHandler : public InterfaceHandler<rt_gui::List,std::string>
{

public:

  typedef std::shared_ptr<ListHandler> Ptr;

  ListHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::List,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TextHandler : public InterfaceHandler<rt_gui::Text,std::string>
{

public:

  typedef std::shared_ptr<TextHandler> Ptr;

  TextHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::Text,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LabelHandler : public InterfaceHandler<rt_gui::Text,std::string>
{

public:

  typedef std::shared_ptr<LabelHandler> Ptr;

  LabelHandler(ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui::Text,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name,true) {}
};

} // namespace


#endif
