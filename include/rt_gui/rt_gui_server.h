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

    double_h_       = std::make_shared<DoubleServerHandler>(window_,ros_node_->getNode(),_ros_services.double_srvs.update,_ros_services.double_srvs.add);
    int_h_          = std::make_shared<IntServerHandler>(window_,ros_node_->getNode(),_ros_services.int_srvs.update,_ros_services.int_srvs.add);
    bool_h_         = std::make_shared<BoolServerHandler>(window_,ros_node_->getNode(),_ros_services.bool_srvs.update,_ros_services.bool_srvs.add);
    list_h_         = std::make_shared<ListServerHandler>(window_,ros_node_->getNode(),_ros_services.list_srvs.update,_ros_services.list_srvs.add);
    trigger_h_      = std::make_shared<TriggerServerHandler>(window_,ros_node_->getNode(),_ros_services.trigger_srvs.update,_ros_services.trigger_srvs.add);

    remove_ = ros_node_->getNode().advertiseService(_ros_services.remove_service, &RtGuiServer::removeWidget, this);

    QObject::connect(this,       SIGNAL(removeWidget(const QString &, const QString &)),
                     window_,    SLOT(removeWidget(const QString &, const QString &)));

    return app_->exec();
  }

  bool removeWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
  {
    emit removeWidget(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
    res.resp = true;
    return res.resp;
  }

signals:
  void removeWidget(const QString &group_name, const QString &data_name);

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

  DoubleServerHandler::Ptr double_h_;
  IntServerHandler::Ptr int_h_;
  BoolServerHandler::Ptr bool_h_;
  ListServerHandler::Ptr list_h_;
  TriggerServerHandler::Ptr trigger_h_;

  ros::ServiceServer remove_;

};


} // namespace


#endif
