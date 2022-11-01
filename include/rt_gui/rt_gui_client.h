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

#ifndef RT_GUI_RT_GUI_CLIENT_H
#define RT_GUI_RT_GUI_CLIENT_H

#include "rt_gui/support/common.h"
#include "rt_gui/support/client.h"
#include <eigen3/Eigen/Core>

namespace rt_gui
{

class RtGuiClient
{
public:


  struct RosHandler {
    std::unique_ptr<RosNode> ros_node_;
    IntHandler::Ptr int_h_;
    DoubleHandler::Ptr double_h_;
    BoolHandler::Ptr bool_h_;
    ListHandler::Ptr list_h_;
    TriggerHandler::Ptr trigger_h_;
    TextHandler::Ptr text_h_;
    ros::ServiceClient remove_;
    bool init_ = false;
  };

  static RtGuiClient& getIstance()
  {
    static RtGuiClient istance;
    return istance;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return ros_handlers_[group_name].int_h_->add(group_name,data_name,min,max,data_ptr,sync);
    }
    else
      return false;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int data, std::function<void(int)> fun, bool sync = true)
  {
    if(check(group_name))
      return ros_handlers_[group_name].int_h_->add(group_name,data_name,min,max,data,fun,sync);
    else
      return false;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
  {
    if(check(group_name))
    {
      int init_value;
      if(loadFromServer(group_name,data_name,init_value) && ros_handlers_[group_name].int_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::vector<int>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && ros_handlers_[group_name].int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i),sync);
      return res;
    }
    else
      return false;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, Eigen::VectorXi* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      std::vector<int> std_v;
      if(load_init_from_server)
        loadFromServer(group_name,data_name,std_v);
      bool res = true;
      for(unsigned int i=0;i<std_v.size();i++) // Copy
        data_ptr->operator[](i) = std_v[i];
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && ros_handlers_[group_name].int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i),sync);
      return res;
    }
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return ros_handlers_[group_name].double_h_->add(group_name,data_name,min,max,data_ptr,sync);
    }
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double data, std::function<void(double)> fun, bool sync = true)
  {
    if(check(group_name))
      return ros_handlers_[group_name].double_h_->add(group_name,data_name,min,max,data,fun,sync);
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::function<void(double)> fun, bool sync = true)
  {
    if(check(group_name))
    {
      int init_value;
      if(loadFromServer(group_name,data_name,init_value) && ros_handlers_[group_name].double_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::vector<double>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && ros_handlers_[group_name].double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i),sync);
      return res;
    }
    else
      return false;
  }

  template<typename Derived>
  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, Eigen::MatrixBase<Derived>* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      std::vector<double> std_v;
      if(load_init_from_server)
        loadFromServer(group_name,data_name,std_v);
      for(unsigned int i=0;i<std_v.size();i++) // Copy
        data_ptr->operator[](i) = std_v[i];
      bool res = true;
      for(unsigned int i=0;i<data_ptr->size();i++)
        res = res && ros_handlers_[group_name].double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i),sync);
      return res;
    }
    else
      return false;
  }

  bool addBool(const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
  {
    if(check(group_name))
    {
      bool init_value;
      if(loadFromServer(group_name,data_name,init_value) && ros_handlers_[group_name].bool_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool addBool(const std::string& group_name, const std::string& data_name, bool* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return ros_handlers_[group_name].bool_h_->add(group_name,data_name,data_ptr,sync);
    }
    else
      return false;
  }

  bool addTrigger(const std::string& group_name, const std::string& data_name, std::function<void()> fun)
  {
    if(check(group_name))
      return ros_handlers_[group_name].trigger_h_->add(group_name,data_name,fun);
    else
      return false;
  }

  bool addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return ros_handlers_[group_name].list_h_->add(group_name,data_name,list,data_ptr,sync);
    }
    else
      return false;
  }

  bool addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check(group_name))
    {
      std::string init_value;
      if(loadFromServer(group_name,data_name,init_value) && ros_handlers_[group_name].list_h_->add(group_name,data_name,list,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool addText(const std::string& group_name, const std::string& data_name, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
  {
    if(check(group_name))
    {
      if(load_init_from_server)
        loadFromServer(group_name,data_name,*data_ptr);
      return ros_handlers_[group_name].text_h_->add(group_name,data_name,data_ptr,sync);
    }
    else
      return false;
  }

  bool addText(const std::string& group_name, const std::string& data_name, std::function<void(std::string)> fun, bool sync = true)
  {
    if(check(group_name))
    {
      std::string init_value;
      if(loadFromServer(group_name,data_name,init_value) && ros_handlers_[group_name].text_h_->add(group_name,data_name,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool remove(const std::string& group_name, const std::string& data_name)
  {
    rt_gui::Void srv;
    srv.request.data_name = data_name;
    srv.request.group_name = group_name;
    return ros_handlers_[group_name].remove_.call(srv);
  }

  void sync()
  {
    for (auto const& rh : ros_handlers_)
    {
      if(rh.second.init_)
      {
        rh.second.double_h_->sync();
        rh.second.int_h_->sync();
        rh.second.bool_h_->sync();
        rh.second.list_h_->sync();
        rh.second.text_h_->sync();
      }
      else {
        ROS_WARN_ONCE("RtGuiClient for %s has not been initialized, please call the init() function before using sync().",rh.first.c_str());
      }
    }
  }

  bool init(ros::NodeHandle& nh, const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    std::string remove_service_name = server_name + "/" + _ros_services.remove_service;
    if(ros::service::waitForService(remove_service_name,timeout))
    {
      ros_handlers_[client_name].remove_         = nh.serviceClient<rt_gui::Void>(remove_service_name);
      ros_handlers_[client_name].bool_h_         = std::make_shared<BoolHandler>   (nh,_ros_services.bool_srvs.add,_ros_services.bool_srvs.update,server_name,client_name);
      ros_handlers_[client_name].list_h_         = std::make_shared<ListHandler>   (nh,_ros_services.list_srvs.add,_ros_services.list_srvs.update,server_name,client_name);
      ros_handlers_[client_name].trigger_h_      = std::make_shared<TriggerHandler>(nh,_ros_services.trigger_srvs.add,_ros_services.trigger_srvs.update,server_name,client_name);
      ros_handlers_[client_name].double_h_       = std::make_shared<DoubleHandler> (nh,_ros_services.double_srvs.add,_ros_services.double_srvs.update,server_name,client_name);
      ros_handlers_[client_name].int_h_          = std::make_shared<IntHandler>    (nh,_ros_services.int_srvs.add,_ros_services.int_srvs.update,server_name,client_name);
      ros_handlers_[client_name].text_h_         = std::make_shared<TextHandler>   (nh,_ros_services.text_srvs.add,_ros_services.text_srvs.update,server_name,client_name);
      ros_handlers_[client_name].init_           = true;
    }
    else
    {
      ROS_WARN_STREAM("RtGuiClient could not find "<< server_name << ", please select the right server_name when calling init().");
      ros_handlers_[client_name].init_ = false;
    }
    return ros_handlers_[client_name].init_;
  }

  bool init(const std::string server_name = RT_GUI_SERVER_NAME, const std::string client_name = RT_GUI_CLIENT_NAME, ros::Duration timeout = ros::Duration(-1))
  {
    // Check if client/group already exists in the ros handlers map
    if(ros_handlers_.count(client_name))
    {
       ROS_WARN("RtGuiClient for client/group %s has been created already, please choose a different name",client_name.c_str());
       return false;
    }
    ros_handlers_[client_name].ros_node_.reset(new RosNode(client_name,_ros_services.n_threads));
    return init(ros_handlers_[client_name].ros_node_->getNode(),server_name,client_name,timeout);
  }

  bool isInitialized(const std::string client_name)
  {
    return ros_handlers_[client_name].init_;
  }

private:

  RtGuiClient()
  {
  }

  bool check(const std::string client_name)
  {
    if(!ros_handlers_[client_name].init_ || !ros_handlers_[client_name].ros_node_ || !ros_handlers_[client_name].ros_node_->initDone())
    {
      ROS_WARN("RtGuiClient for %s is not initialized yet, please call init() function before adding widgets.",client_name.c_str());
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

  std::map<std::string,RosHandler> ros_handlers_;

};


} // namespace


#endif
