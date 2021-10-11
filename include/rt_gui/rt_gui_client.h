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

#include <support/common.h>
#include <support/client.h>
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

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr)
  {
    int_h_->add(group_name,data_name,min,max,data_ptr);
  }

  void addInt(const std::string& group_name, const std::string& data_name, const int& min, const int& max, SetIntHandler::funct_t fun)
  {
    set_int_h_->add<int>(group_name,data_name,min,max,fun);
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::vector<int>* data_ptr)
  {
    for(unsigned int i=0;i<data_ptr->size();i++)
      int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const int& min, const int& max, Eigen::VectorXi* data_ptr)
  {
    for(unsigned int i=0;i<data_ptr->size();i++)
      int_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i));
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
  {
    double_h_->add(group_name,data_name,min,max,data_ptr);
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, SetDoubleHandler::funct_t fun)
  {
    set_double_h_->add<double>(group_name,data_name,min,max,fun);
    getchar();
  }

  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, std::vector<double>* data_ptr)
  {
    for(unsigned int i=0;i<data_ptr->size();i++)
      double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->at(i));
  }

  template<typename Derived>
  void addDouble(const std::string& group_name, const std::string& data_name, const double& min, const double& max, Eigen::MatrixBase<Derived>* data_ptr)
  {
    for(unsigned int i=0;i<data_ptr->size();i++)
      double_h_->add(group_name,data_name+"["+std::to_string(i)+"]",min,max,&data_ptr->operator[](i));
  }

  void addBool(const std::string& group_name, const std::string& data_name, bool* data_ptr)
  {
    bool_h_->add(group_name,data_name,data_ptr);
  }

  void addTrigger(const std::string& group_name, const std::string& data_name, TriggerHandler::funct_t fun)
  {
    trigger_h_->add(group_name,data_name,fun);
  }

  void addList(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr)
  {
    list_h_->add(group_name,data_name,list,data_ptr);
  }

  void remove(const std::string& group_name, const std::string& data_name)
  {
    rt_gui::Void srv;
    srv.request.data_name = data_name;
    srv.request.group_name = group_name;
    remove_.call(srv);
  }

  void sync()
  {
    double_h_->sync();
    int_h_->sync();
    bool_h_->sync();
    list_h_->sync();
    //trigger_h_->sync();
  }

private:

  RtGuiClient()
  {
    std::string ros_node_name = RT_GUI_CLIENT_NAME;
    ros_node_.reset(new RosNode(ros_node_name,_ros_services.n_threads));
    double_h_       = std::make_shared<DoubleHandler>(ros_node_->getNode(),_ros_services.double_slider.add,_ros_services.double_slider.update);
    set_double_h_   = std::make_shared<SetDoubleHandler>(ros_node_->getNode(),_ros_services.double_slider.add,_ros_services.double_slider.set);
    int_h_          = std::make_shared<IntHandler>(ros_node_->getNode(),_ros_services.int_slider.add,_ros_services.int_slider.update);
    set_int_h_      = std::make_shared<SetIntHandler>(ros_node_->getNode(),_ros_services.int_slider.add,_ros_services.int_slider.set);
    bool_h_         = std::make_shared<BoolHandler>(ros_node_->getNode(),_ros_services.radio_button.add,_ros_services.radio_button.update);
    list_h_         = std::make_shared<ListHandler>(ros_node_->getNode(),_ros_services.combo_box.add,_ros_services.combo_box.update);
    trigger_h_      = std::make_shared<TriggerHandler>(ros_node_->getNode(),_ros_services.button.add,_ros_services.button.update);

    remove_ = ros_node_->getNode().serviceClient<rt_gui::Void>("/" RT_GUI_SERVER_NAME "/"+_ros_services.remove_service);
  }

  ~RtGuiClient()
  {
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;
  DoubleHandler::Ptr double_h_;
  SetDoubleHandler::Ptr set_double_h_;
  IntHandler::Ptr int_h_;
  SetIntHandler::Ptr set_int_h_;
  BoolHandler::Ptr bool_h_;
  ListHandler::Ptr list_h_;
  TriggerHandler::Ptr trigger_h_;

  ros::ServiceClient remove_;

};


} // namespace


#endif
