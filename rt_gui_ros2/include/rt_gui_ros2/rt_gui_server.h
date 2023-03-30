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

#ifndef RT_GUI_ROS2_RT_GUI_SERVER_H
#define RT_GUI_ROS2_RT_GUI_SERVER_H

#include <rt_gui_ros2/support/server.h>

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
    app_ = new QApplication(argc,argv);
    if(argv[1]!=nullptr)
      ros_server_node_.init(argv[1]);
    else
      ros_server_node_.init();
    return app_->exec();
  }

  void stop()
  {
    app_->exit(0);
  }

private:

  RtGuiServer()
  {
  }

  RtGuiServer(const RtGuiServer&)= delete;
  RtGuiServer& operator=(const RtGuiServer&)= delete;

  QApplication* app_;
  RosServerNode ros_server_node_;

};


} // namespace


#endif
