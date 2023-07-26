/*
 * Copyright 2019 Gennaro Raiola
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Gennaro Raiola
 */

#ifndef RT_GUI_ROS_RT_GUI_CLIENT_H
#define RT_GUI_ROS_RT_GUI_CLIENT_H

#include <rt_gui_ros/support/client.h>
#include <eigen3/Eigen/Core>

#include <queue>

namespace rt_gui
{

class RtGuiClient
{
public:

  typedef std::function<bool()> fun_t;

  static RtGuiClient& getIstance()
  {
    static RtGuiClient istance;
    return istance;
  }

  ~RtGuiClient()
  {
    ack_.join();
    requests_.join();
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const int&,const int&,int*,bool,bool)>(&RtGuiClient::_addInt)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const int&, const int&,std::function<void(int)>,bool)>(&RtGuiClient::_addInt)
              ,this,group_name,data_name,min,max,fun,sync));
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int init_value, std::function<void(int)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const int&,const int&,int,std::function<void(int)>,bool)>(&RtGuiClient::_addInt)
              ,this,group_name,data_name,min,max,init_value,fun,sync));
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::vector<int>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const int&,const int&,std::vector<int>*,bool,bool)>(&RtGuiClient::_addInt)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, Eigen::VectorXi* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const int&,const int&,Eigen::VectorXi*,bool,bool)>(&RtGuiClient::_addInt)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const double&,const double&,double*,bool,bool)>(&RtGuiClient::_addDouble)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::function<void(double)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const double&,const double&,std::function<void(double)>,bool)>(&RtGuiClient::_addDouble)
              ,this,group_name,data_name,min,max,fun,sync));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double init_value, std::function<void(double)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const double&,const double&,double,std::function<void(double)>,bool)>(&RtGuiClient::_addDouble)
              ,this,group_name,data_name,min,max,init_value,fun,sync));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::vector<double>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const double&,const double&,std::vector<double>*,bool,bool)>(&RtGuiClient::_addDouble)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  template<typename Derived>
  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, Eigen::MatrixBase<Derived>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const double&,const double&,Eigen::MatrixBase<Derived>*,bool,bool)>(&RtGuiClient::_addDouble)
              ,this,group_name,data_name,min,max,data_ptr,sync,load_init_from_server));
  }

  void addBool(const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::function<void(bool)>,bool)>(&RtGuiClient::_addBool)
              ,this,group_name,data_name,fun,sync));
  }

  void addBool(const std::string& group_name, const std::string& data_name, bool init_value, std::function<void(bool)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,bool,std::function<void(bool)>,bool)>(&RtGuiClient::_addBool)
              ,this,group_name,data_name,init_value,fun,sync));
  }

  void addBool(const std::string& group_name, const std::string& data_name, bool* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,bool*,bool,bool)>(&RtGuiClient::_addBool)
              ,this,group_name,data_name,data_ptr,sync,load_init_from_server));
  }

  void addTrigger(const std::string& group_name, const std::string& data_name, std::function<void()> fun)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::function<void()>)>(&RtGuiClient::_addTrigger)
              ,this,group_name,data_name,fun));
  }

  void addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const std::vector<std::string>&,std::string*,bool,bool)>(&RtGuiClient::_addList)
              ,this,group_name,data_name,list,data_ptr,sync,load_init_from_server));
  }

  void addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,const std::vector<std::string>&,std::function<void(std::string)>,bool)>(&RtGuiClient::_addList)
              ,this,group_name,data_name,list,fun,sync));
  }

  void addList(const std::string& group_name, const std::string& data_name, std::string init_value, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::string,const std::vector<std::string>&,std::function<void(std::string)>,bool)>(&RtGuiClient::_addList)
              ,this,group_name,data_name,init_value,list,fun,sync));
  }

  void addText(const std::string& group_name, const std::string& data_name, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::string*,bool,bool)>(&RtGuiClient::_addText)
              ,this,group_name,data_name,data_ptr,sync,load_init_from_server));
  }

  void addText(const std::string& group_name, const std::string& data_name, std::function<void(std::string)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::function<void(std::string)>,bool)>(&RtGuiClient::_addText)
              ,this,group_name,data_name,fun,sync));
  }

  void addText(const std::string& group_name, const std::string& data_name, std::string init_value, std::function<void(std::string)> fun, bool sync = true)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::string,std::function<void(std::string)>,bool)>(&RtGuiClient::_addText)
              ,this,group_name,data_name,init_value,fun,sync));
  }

  void addLabel(const std::string& group_name, const std::string& data_name, std::string* data_ptr, bool load_init_from_server = false)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&,std::string*,bool)>(&RtGuiClient::_addLabel)
              ,this,group_name,data_name,data_ptr,load_init_from_server));
  }

  void remove(const std::string& group_name, const std::string& data_name)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&,const std::string&)>(&RtGuiClient::_remove)
              ,this,group_name,data_name));
  }

  void remove(const std::string& group_name)
  {
    collector_.push(std::bind(static_cast<bool(RtGuiClient::*)(const std::string&)>(&RtGuiClient::_remove)
              ,this,group_name));
  }

  void sync()
  {
    if(init_)
    {
      double_h_->sync();
      int_h_->sync();
      bool_h_->sync();
      list_h_->sync();
      text_h_->sync();
      label_h_->sync();
    }
    else {
      ROS_WARN_ONCE("RtGuiClient has not been initialized, please be sure to call the init() function before using sync().");
    }
  }

  void init(ros::NodeHandle& nh, const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    if(!ack_.joinable() && !requests_.joinable())
    {
      ack_ = std::thread(std::bind(&RtGuiClient::_init,this,nh,server_name,client_name,timeout));
      requests_ = std::thread(&RtGuiClient::_requests,this);
    }
    else {
       ROS_WARN_STREAM("RtGuiClient has been already initialized! You are calling the same instance twice!");
    }
  }

  void init(const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    if(ros_node_==nullptr)
      ros_node_.reset(new RosNode(client_name,_ros_services.n_threads));
    init(ros_node_->getNode(),server_name,client_name,timeout);
  }

  bool isInitialized()
  {
    return init_;
  }

private:

  void _init(ros::NodeHandle& nh, const std::string server_name, const std::string client_name, ros::Duration timeout)
  {
    std::string remove_service_name = server_name + "/" + _ros_services.remove_service;
    ros::NodeHandle global_nh;

    if(ros::service::waitForService(remove_service_name,timeout))
    {
      remove_         = global_nh.serviceClient<rt_gui::Void>(remove_service_name);
      bool_h_         = std::make_shared<BoolHandler>   (nh,_ros_services.bool_srvs.add,_ros_services.bool_srvs.update,_ros_services.bool_srvs.feedback,server_name,client_name);
      list_h_         = std::make_shared<ListHandler>   (nh,_ros_services.list_srvs.add,_ros_services.list_srvs.update,_ros_services.list_srvs.feedback,server_name,client_name);
      trigger_h_      = std::make_shared<TriggerHandler>(nh,_ros_services.trigger_srvs.add,_ros_services.trigger_srvs.update,_ros_services.trigger_srvs.feedback,server_name,client_name);
      double_h_       = std::make_shared<DoubleHandler> (nh,_ros_services.double_srvs.add,_ros_services.double_srvs.update,_ros_services.double_srvs.feedback,server_name,client_name);
      int_h_          = std::make_shared<IntHandler>    (nh,_ros_services.int_srvs.add,_ros_services.int_srvs.update,_ros_services.int_srvs.feedback,server_name,client_name);
      text_h_         = std::make_shared<TextHandler>   (nh,_ros_services.text_srvs.add,_ros_services.text_srvs.update,_ros_services.text_srvs.feedback,server_name,client_name);
      label_h_        = std::make_shared<LabelHandler>  (nh,_ros_services.label_srvs.add,_ros_services.label_srvs.update,_ros_services.label_srvs.feedback,server_name,client_name);
      init_           = true;
    }
    else
    {
      ROS_WARN_STREAM("RtGuiClient could not find "<< server_name << ", please select the right server_name when calling init().");
      init_ = false;
    }
  }

  void _requests()
  {
    while(ros::ok())
    {
      while (!collector_.empty())
        if(collector_.front()())
          collector_.pop();
      ros::Rate(1).sleep();
    }
  }

  RtGuiClient()
  {
    init_ = false;
  }

  bool check()
  {
    if(!init_ || !ros_node_ || !ros_node_->initDone())
      return false;
    else
      return true;
  }

  template<class data_t>
  bool loadFromServer(const std::string& group_name, const std::string& data_name, data_t& value)
  {
    bool res = true;
    data_t init_value;
    std::string ns = "/"+group_name+"/"+data_name;
    if (!ros::param::get(ns, init_value))
    {
      ROS_WARN("No initial value given in namespace %s",ns.c_str());
      res = false;
    }
    else
    {
      value = init_value;
      res = true;
    }
    return res;
  }

  bool _addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return int_h_->add(group_name,data_name,min,max,data_ptr,sync);
    }
    else
      return false;
  }

  bool _addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
  {
    if(check())
    {
      int init_value;
      if(loadFromServer(group_name,data_name,init_value) && int_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int init_value, std::function<void(int)> fun, bool sync = true)
  {
    if(check())
    {
      if(int_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::vector<int>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i),sync);
      return res;
    }
    else
      return false;
  }

  bool _addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, Eigen::VectorXi* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      std::vector<int> std_v;
      if(load_init_from_server)
        loadFromServer(group_name,data_name,std_v);
      bool res = true;
      for(unsigned int i=0;i<std_v.size();i++) // Copy
        data_ptr->operator[](i) = std_v[i];
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i),sync);
      return res;
    }
    else
      return false;
  }

  bool _addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return double_h_->add(group_name,data_name,min,max,data_ptr,sync);
    }
    else
      return false;
  }

  bool _addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::function<void(double)> fun, bool sync = true)
  {
    if(check())
    {
      double init_value;
      if(loadFromServer(group_name,data_name,init_value) && double_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double init_value, std::function<void(double)> fun, bool sync = true)
  {
    if(check())
    {
      if(double_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::vector<double>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i),sync);
      return res;
    }
    else
      return false;
  }

  template<typename Derived>
  bool _addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, Eigen::MatrixBase<Derived>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      std::vector<double> std_v;
      if(load_init_from_server)
        loadFromServer(group_name,data_name,std_v);
      for(unsigned int i=0;i<std_v.size();i++) // Copy
        data_ptr->operator[](i) = std_v[i];
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i),sync);
      return res;
    }
    else
      return false;
  }

  bool _addBool(const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
  {
    if(check())
    {
      bool init_value;
      if(loadFromServer(group_name,data_name,init_value) && bool_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addBool(const std::string& group_name, const std::string& data_name, bool init_value, std::function<void(bool)> fun, bool sync = true)
  {
    if(check())
    {
      if(bool_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addBool(const std::string& group_name, const std::string& data_name, bool* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return bool_h_->add(group_name,data_name,data_ptr,sync);
    }
    else
      return false;
  }

  bool _addTrigger(const std::string& group_name, const std::string& data_name, std::function<void()> fun)
  {
    if(check())
      return trigger_h_->add(group_name,data_name,fun);
    else
      return false;
  }

  bool _addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return list_h_->add(group_name,data_name,list,data_ptr,sync);
    }
    else
      return false;
  }

  bool _addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check())
    {
      std::string init_value;
      if(loadFromServer(group_name,data_name,init_value) && list_h_->add(group_name,data_name,list,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addList(const std::string& group_name, const std::string& data_name, std::string init_value, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check())
    {
      if(list_h_->add(group_name,data_name,list,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addText(const std::string& group_name, const std::string& data_name, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return text_h_->add(group_name,data_name,data_ptr,sync);
    }
    else
      return false;
  }

  bool _addText(const std::string& group_name, const std::string& data_name, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check())
    {
      std::string init_value;
      if(loadFromServer(group_name,data_name,init_value) && text_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addText(const std::string& group_name, const std::string& data_name, std::string init_value, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check())
    {
      if(text_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool _addLabel(const std::string& group_name, const std::string& data_name, std::string* data_ptr, bool load_init_from_server = false)
  {
    if(check())
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return label_h_->add(group_name,data_name,data_ptr,false,true);
    }
    else
      return false;
  }

  bool _remove(const std::string& group_name, const std::string& data_name)
  {
    rt_gui_msgs::Void srv;
    srv.request.data_name = data_name;
    srv.request.group_name = group_name;
    return remove_.call(srv);
  }

  bool _remove(const std::string& group_name)
  {
    rt_gui_msgs::Void srv;
    srv.request.data_name = "";
    srv.request.group_name = group_name;
    return remove_.call(srv);
  }

  void sync()
  {
    if(init_)
    {
      double_h_->sync();
      int_h_->sync();
      bool_h_->sync();
      list_h_->sync();
      text_h_->sync();
      label_h_->sync();
    }
    else {
      ROS_WARN_ONCE("RtGuiClient has not been initialized, please call the init() function before using sync().");
    }
  }

  bool init(ros::NodeHandle& nh, const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    std::string remove_service_name = server_name + "/" + _ros_services.remove_service;
    ros::NodeHandle global_nh;
    if(ros::service::waitForService(remove_service_name,timeout))
    {
      remove_         = global_nh.serviceClient<rt_gui_msgs::Void>(remove_service_name);
      bool_h_         = std::make_shared<BoolHandler>   (nh,_ros_services.bool_srvs.add,_ros_services.bool_srvs.update,_ros_services.bool_srvs.feedback,server_name,client_name);
      list_h_         = std::make_shared<ListHandler>   (nh,_ros_services.list_srvs.add,_ros_services.list_srvs.update,_ros_services.list_srvs.feedback,server_name,client_name);
      trigger_h_      = std::make_shared<TriggerHandler>(nh,_ros_services.trigger_srvs.add,_ros_services.trigger_srvs.update,_ros_services.trigger_srvs.feedback,server_name,client_name);
      double_h_       = std::make_shared<DoubleHandler> (nh,_ros_services.double_srvs.add,_ros_services.double_srvs.update,_ros_services.double_srvs.feedback,server_name,client_name);
      int_h_          = std::make_shared<IntHandler>    (nh,_ros_services.int_srvs.add,_ros_services.int_srvs.update,_ros_services.int_srvs.feedback,server_name,client_name);
      text_h_         = std::make_shared<TextHandler>   (nh,_ros_services.text_srvs.add,_ros_services.text_srvs.update,_ros_services.text_srvs.feedback,server_name,client_name);
      label_h_        = std::make_shared<LabelHandler>  (nh,_ros_services.label_srvs.add,_ros_services.label_srvs.update,_ros_services.label_srvs.feedback,server_name,client_name);
      init_           = true;
    }
    else
    {
      ROS_WARN_STREAM("RtGuiClient could not find "<< server_name << ", please select the right server_name when calling init().");
      init_ = false;
    }
    return init_;
  }

  bool init(const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    ros_node_.reset(new RosNode(client_name,_ros_services.n_threads));
    return init(ros_node_->getNode(),server_name,client_name,timeout);
  }

  bool isInitialized()
  {
    return init_;
  }

private:

  RtGuiClient()
  {
    init_ = false;
  }

  bool check()
  {
    if(!init_ || !ros_node_ || !ros_node_->initDone())
    {
      ROS_WARN("RtGuiClient not initialized, please call init() function before adding widgets.");
      return false;
    }
    else
      return true;
  }

  template<class data_t>
  bool loadFromServer(const std::string& group_name, const std::string& data_name, data_t& value)
  {
    bool res = true;
    data_t init_value;
    std::string ns = "/"+group_name+"/"+data_name;
    if (!ros::param::get(ns, init_value))
    {
      ROS_WARN("No initial value given in namespace %s",ns.c_str());
      res = false;
    }
    else
    {
      value = init_value;
      res = true;
    }
    return res;
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;
  IntHandler::Ptr int_h_;
  DoubleHandler::Ptr double_h_;
  BoolHandler::Ptr bool_h_;
  ListHandler::Ptr list_h_;
  TriggerHandler::Ptr trigger_h_;
  TextHandler::Ptr text_h_;
  LabelHandler::Ptr label_h_;
  ros::ServiceClient remove_;
  std::atomic<bool> init_;
  std::thread ack_;
  std::thread requests_;
  std::queue<fun_t> collector_;

};


} // namespace


#endif
