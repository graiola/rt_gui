#ifndef RT_GUI_ROS2_SUPPORT_CLIENT_H
#define RT_GUI_ROS2_SUPPORT_CLIENT_H

#include <rt_gui_ros2/support/ros_node.h>
#include <rt_gui_core/support/common.h>
#include <type_traits>

namespace rt_gui
{

using namespace std::placeholders;

template<class srv_t, class data_t>
class InterfaceHandler
{

public:

  typedef std::shared_ptr<InterfaceHandler> Ptr;

  typedef std::function<void(data_t)> fun_t;

  InterfaceHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name, bool feedback = false)
  {
    add_srv_       = add_srv;
    update_srv_    = update_srv;
    feedback_srv_  = feedback_srv;
    server_name_   = server_name;
    client_name_   = client_name;
    update_        = node->create_service<srv_t>("/"+client_name+"/"+update_srv, std::bind(&InterfaceHandler::update, this, _1, _2));
    add_           = node->create_client<srv_t>("/"+server_name+"/"+add_srv);
    feedback_      = node->create_client<srv_t>("/"+server_name+"/"+feedback_srv);
    stop_feedback_thread_ = false;
    if(feedback)
      feedback_thread_ = std::make_shared<std::thread>(&InterfaceHandler::feedbackLoop,this);

  }

  virtual ~InterfaceHandler() { stop_feedback_thread_ = true; if(feedback_thread_!=nullptr) feedback_thread_->join(); }

  bool update(typename srv_t::Request::Ptr req, typename srv_t::Response::Ptr res)
  {
    res->resp = updateBuffer(req->group_name,req->data_name,req->value);
    return true;
  }

  bool add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t* data_ptr, bool sync)
  {
    typename srv_t::Request srv_req;
    srv_req.value = *data_ptr;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    for(unsigned int i=0;i<list.size();i++)
      srv_req.list.push_back(list[i]);
    return addRawData(group_name,data_name,data_ptr,srv_req,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t data, fun_t fun, bool sync)
  {
    typename srv_t::Request srv_req;
    srv_req.value = data;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    for(unsigned int i=0;i<list.size();i++)
      srv_req.list.push_back(list[i]);
    return addCallback(group_name,data_name,data,fun,srv_req,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, bool sync, bool read_only = false)
  {
    typename srv_t::Request srv_req;
    srv_req.value = *data_ptr;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    return addRawData(group_name,data_name,data_ptr,srv_req,sync,read_only);
  }

  bool add(const std::string& group_name, const std::string& data_name, data_t data, fun_t fun, bool sync)
  {
    typename srv_t::Request srv_req;
    srv_req.value = data;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    return addCallback(group_name,data_name,data,fun,srv_req,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t* data_ptr, bool sync)
  {
    typename srv_t::Request srv_req;
    srv_req.min = min;
    srv_req.max = max;
    srv_req.value = *data_ptr;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    return addRawData(group_name,data_name,data_ptr,srv_req,sync);
  }

  bool add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t data, fun_t fun, bool sync)
  {
    typename srv_t::Request srv_req;
    srv_req.min = min;
    srv_req.max = max;
    srv_req.value = data;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    return addCallback(group_name,data_name,data,fun,srv_req,sync);
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
    //if(feedback_->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
    //{
      typename srv_t::Request srv_req;
      for(auto tmp_map : buffer_.getBuffer())
      {
        auto group_name = tmp_map.first.first;
        auto data_name = tmp_map.first.second;
        if(buffer_.isDataChanged(group_name,data_name))
        {
          data_t value;
          buffer_.getValue(group_name,data_name,value);
          srv_req.value       = value;
          srv_req.client_name = client_name_;
          srv_req.group_name  = group_name;
          srv_req.data_name   = data_name;
          auto result = feedback_->async_send_request(std::make_shared<typename srv_t::Request>(srv_req));
          result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));
          //if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
          //     return false;
        }
      }
      return true;
    //}
    //else
    //  return false;
  }

  void feedbackLoop()
  {
    while(!stop_feedback_thread_)
    {
      sync_mtx_.lock();
      if(!feedback())
      {
         RCLCPP_WARN(node_->get_logger(),"RtGuiServer::feedback service is not available!");
         stop_feedback_thread_ = true;
      }
      sync_mtx_.unlock();
      std::this_thread::sleep_for (std::chrono::seconds(1));
    }
  }

  bool addRawData(const std::string& group_name, const std::string& data_name, data_t* data_ptr, typename srv_t::Request& srv_req, bool sync, bool read_only = false)
  {
    //if(this->add_->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
    //{
      auto result = add_->async_send_request(std::make_shared<typename srv_t::Request>(srv_req));
      result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));
      //if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
      //{
        buffer_.add(group_name,data_name,data_ptr,sync,read_only);
      //}
      //else
      //{
      //  return false;
      //}
    //}
    //else
    //{
    //  //ROS_WARN("RtGuiServer::add service is not available!");
    //  return false;
    //}
    return true;
  }

  bool addCallback(const std::string& group_name, const std::string& data_name, data_t data, std::function<void(data_t)> fun, typename srv_t::Request& srv, bool sync)
  {
    //if(add_->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
    //{
      auto result = add_->async_send_request(std::make_shared<typename srv_t::Request>(srv));
      result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));

      //if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
      //{
           buffer_.add(group_name,data_name,data,fun,sync);
      //}
      //else
      //{
      //    //ROS_WARN("RtGuiServer::add call response is false!");
      //    return false;
      //}
    //}
    //else
    //{
    //  //ROS_WARN("RtGuiServer::add service is not available!");
    //  return false;
    //}
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

  typename rclcpp::Service<srv_t>::SharedPtr update_;
  typename rclcpp::Client<srv_t>::SharedPtr feedback_;
  typename rclcpp::Client<srv_t>::SharedPtr add_;
  CallbackBuffer<data_t> buffer_;
  std::mutex sync_mtx_;

  std::shared_ptr<std::thread> feedback_thread_;
  bool stop_feedback_thread_;

  std::shared_ptr<rclcpp::Node> node_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerHandler
{

public:

  typedef std::shared_ptr<TriggerHandler> Ptr;

  typedef std::function<void()> fun_t;

  TriggerHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
  {
    add_srv_       = add_srv;
    update_srv_    = update_srv;
    feedback_srv_  = feedback_srv;
    server_name_   = server_name;
    client_name_   = client_name;
    node_          = node;
    update_        = node_->create_service<rt_gui_msgs::srv::Void>("/"+client_name+"/"+update_srv_, std::bind(&TriggerHandler::update, this, _1, _2));
    add_           = node_->create_client<rt_gui_msgs::srv::Void>("/"+server_name+"/"+add_srv);
    feedback_      = node_->create_client<rt_gui_msgs::srv::Void>("/"+server_name+"/"+feedback_srv);
  }

  bool add(const std::string& group_name, const std::string& data_name, fun_t fun)
  {
    rt_gui_msgs::srv::Void::Request srv_req;
    assert(fun);
    funs_[key_t(group_name,data_name)] = fun;
    srv_req.client_name = client_name_;
    srv_req.group_name = group_name;
    srv_req.data_name = data_name;
    //if(this->add_->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
    //{
      auto result = add_->async_send_request(std::make_shared<rt_gui_msgs::srv::Void::Request>(srv_req));
      result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));
      //if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
      //     return false;
    //}
    //else
    //{
    //  //ROS_WARN("RtGuiServer::add service is not available!");
    //  return false;
    //}
    return true;
  }

  bool update(rt_gui_msgs::srv::Void::Request::Ptr req, rt_gui_msgs::srv::Void::Response::Ptr res)
  {
    funs_[key_t(req->group_name,req->data_name)]();
    res->resp = true;
    return res->resp;
  }

protected:

  std::string update_srv_;
  std::string add_srv_;
  std::string feedback_srv_;
  std::string server_name_;
  std::string client_name_;

  std::map<key_t,fun_t> funs_;
  rclcpp::Service<rt_gui_msgs::srv::Void>::SharedPtr update_;
  rclcpp::Client<rt_gui_msgs::srv::Void>::SharedPtr feedback_;
  rclcpp::Client<rt_gui_msgs::srv::Void>::SharedPtr add_;

  std::shared_ptr<rclcpp::Node> node_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntHandler : public InterfaceHandler<rt_gui_msgs::srv::Int,int>
{

public:

  typedef std::shared_ptr<IntHandler> Ptr;

  IntHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::Int,int>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleHandler : public InterfaceHandler<rt_gui_msgs::srv::Double,double>
{

public:

  typedef std::shared_ptr<DoubleHandler> Ptr;

  DoubleHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::Double,double>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolHandler : public InterfaceHandler<rt_gui_msgs::srv::Bool,bool>
{

public:

  typedef std::shared_ptr<BoolHandler> Ptr;

  BoolHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::Bool,bool>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListHandler : public InterfaceHandler<rt_gui_msgs::srv::List,std::string>
{

public:

  typedef std::shared_ptr<ListHandler> Ptr;

  ListHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::List,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TextHandler : public InterfaceHandler<rt_gui_msgs::srv::Text,std::string>
{

public:

  typedef std::shared_ptr<TextHandler> Ptr;

  TextHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::Text,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LabelHandler : public InterfaceHandler<rt_gui_msgs::srv::Text,std::string>
{

public:

  typedef std::shared_ptr<LabelHandler> Ptr;

  LabelHandler(std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv, std::string server_name, std::string client_name)
    :InterfaceHandler<rt_gui_msgs::srv::Text,std::string>(node,add_srv,update_srv,feedback_srv,server_name,client_name,true) {}
};

} // namespace


#endif
