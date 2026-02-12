/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef RT_GUI_ROS_RT_GUI_SERVER_H
#define RT_GUI_ROS_RT_GUI_SERVER_H

#include <rt_gui_ros/support/server.h>

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
