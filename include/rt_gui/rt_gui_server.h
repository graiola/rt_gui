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

#ifndef RT_GUI_RT_GUI_SERVER_H
#define RT_GUI_RT_GUI_SERVER_H

#include <support/common.h>
#include <support/server.h>

namespace rt_gui
{

class RtGuiServer : public QObject
{

  Q_OBJECT

public:

  static RtGuiServer& getIstance()
  {
    static RtGuiServer istance;
    return istance;
  }

  int run(int argc, char *argv[])
  {
    std::string ros_node_name = RT_GUI_SERVER_NAME;
    ros_node_.reset(new RosNode(ros_node_name,_ros_services.n_threads));

    app_ = new QApplication(argc,argv);
    window_ = new Window(QString::fromStdString(ros_node_name));

    slider_m_ = std::make_shared<SliderServerManager>(window_,ros_node_->getNode(),_ros_services.slider.update,_ros_services.slider.add);
    radio_m_  = std::make_shared<RadioButtonServerManager>(window_,ros_node_->getNode(),_ros_services.radio_button.update,_ros_services.radio_button.add);
    combo_m_  = std::make_shared<ComboBoxServerManager>(window_,ros_node_->getNode(),_ros_services.combo_box.update,_ros_services.combo_box.add);

    return app_->exec();
  }

private:

  RtGuiServer()
  {
  }

  ~RtGuiServer()
  {
  }

  RtGuiServer(const RtGuiServer&)= delete;
  RtGuiServer& operator=(const RtGuiServer&)= delete;

  Window* window_;
  QApplication* app_;
  std::unique_ptr<RosNode> ros_node_;

  SliderServerManager::Ptr slider_m_;
  RadioButtonServerManager::Ptr radio_m_;
  ComboBoxServerManager::Ptr combo_m_;
};


} // namespace


#endif
