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

  static RtGuiClient& getIstance()
  {
    static RtGuiClient istance;
    return istance;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int data, std::function<void(int)> fun, bool sync = true)
  {
    if(check())
      return int_h_->add(group_name,data_name,min,max,data,fun,sync);
    else
      return false;
  }

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
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

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::vector<int>* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, Eigen::VectorXi* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double data, std::function<void(double)> fun, bool sync = true)
  {
    if(check())
      return double_h_->add(group_name,data_name,min,max,data,fun,sync);
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::function<void(double)> fun, bool sync = true)
  {
    if(check())
    {
      int init_value;
      if(loadFromServer(group_name,data_name,init_value) && double_h_->add(group_name,data_name,min,max,init_value,fun,sync))
        return true;
      else
        return false;
    }
    else
      return false;
  }

  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::vector<double>* data_ptr, bool sync = true, bool load_init_from_server = false)
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
  bool addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, Eigen::MatrixBase<Derived>* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addBool(const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
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

  bool addBool(const std::string& group_name, const std::string& data_name, bool* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addTrigger(const std::string& group_name, const std::string& data_name, std::function<void()> fun)
  {
    if(check())
      return trigger_h_->add(group_name,data_name,fun);
    else
      return false;
  }

  bool addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr, bool sync = true, bool load_init_from_server = false)
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

  bool addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
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

  bool remove(const std::string& group_name, const std::string& data_name)
  {
    rt_gui::Void srv;
    srv.request.data_name = data_name;
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
    }
    else {
      ROS_WARN_ONCE("RtGuiClient has not been initialized, please call the init() function before using sync().");
    }
  }

  bool init(const std::string ros_namespace = RT_GUI_NAMESPACE, ros::Duration timeout = ros::Duration(-1))
  {
    std::string ros_node_name = ros_namespace + "_client";
    ros_node_.reset(new RosNode(ros_node_name,_ros_services.n_threads));

    std::string remove_service_name = "/"+ros_namespace+"_server/"+_ros_services.remove_service;
    if(ros::service::waitForService(remove_service_name,timeout))
    {
      remove_         = ros_node_->getNode().serviceClient<rt_gui::Void>(remove_service_name);
      bool_h_         = std::make_shared<BoolHandler>(ros_node_->getNode(),_ros_services.bool_srvs.add,_ros_services.bool_srvs.update,ros_namespace);
      list_h_         = std::make_shared<ListHandler>(ros_node_->getNode(),_ros_services.list_srvs.add,_ros_services.list_srvs.update,ros_namespace);
      trigger_h_      = std::make_shared<TriggerHandler>(ros_node_->getNode(),_ros_services.trigger_srvs.add,_ros_services.trigger_srvs.update,ros_namespace);
      double_h_       = std::make_shared<DoubleHandler>(ros_node_->getNode(),_ros_services.double_srvs.add,_ros_services.double_srvs.update,ros_namespace);
      int_h_          = std::make_shared<IntHandler>(ros_node_->getNode(),_ros_services.int_srvs.add,_ros_services.int_srvs.update,ros_namespace);
      init_           = true;
    }
    else
    {
      ROS_WARN_STREAM("RtGuiClient could not find "<< ros_namespace+"_server"<< ", please select the right ros_namespace when calling init().");
      init_ = false;
    }
    return init_;
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
  ros::ServiceClient remove_;
  bool init_;

};


} // namespace


#endif
