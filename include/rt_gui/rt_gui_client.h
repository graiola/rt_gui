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

  void addSlider(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
  {
    slider_m_->add(group_name,data_name,min,max,data_ptr);
  }

  void addRadioButton(const std::string& group_name, const std::string& data_name, bool* data_ptr)
  {
    radio_m_->add(group_name,data_name,data_ptr);
  }

  void addCheckBox(const std::string& group_name, const std::string& data_name, bool* data_ptr)
  {

  }

  void addComboBox(const std::string& group_name, const std::string& data_name, std::vector<std::string>* data_ptr)
  {

  }

  void sync()
  {
     slider_m_->sync();
     radio_m_->sync();
  }

private:

  RtGuiClient()
  {
    std::string ros_node_name = RT_GUI_CLIENT_NAME;
    ros_node_.reset(new RosNode(ros_node_name,_ros_services.n_threads));
    slider_m_ = std::make_shared<SliderClientManager>(ros_node_->getNode(),_ros_services.slider.add,_ros_services.slider.update);
    radio_m_  = std::make_shared<RadioButtonClientManager>(ros_node_->getNode(),_ros_services.radio_button.add,_ros_services.radio_button.update);
  }

  ~RtGuiClient()
  {
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;
  SliderClientManager::Ptr slider_m_;
  RadioButtonClientManager::Ptr radio_m_;

};


} // namespace


#endif
