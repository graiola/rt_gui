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

#include <rt_gui/common.h>

#include <qt_utils/window.h>
#include <rt_gui/addSlider.h>
#include <rt_gui/sync.h>

namespace rt_gui
{

class RtGuiServer : QObject
{

  Q_OBJECT

public:

  static RtGuiServer& getIstance()
  {
    static RtGuiServer istance;
    return istance;
  }

  bool addSlider(addSlider::Request  &req, addSlider::Response &res)
  {

    emit addSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max);

    // FIXME add a proper error handling
    res.resp = true;

    return res.resp;
  }

  bool sync()
  {
    // Call the client ros service
  }

  int run(int argc, char *argv[])
  {
    std::string ros_node_name = RT_GUI_SERVER_NAME;
    ros_node_.reset(new RosNode(ros_node_name));

    app_ = std::make_shared<QApplication>(argc,argv);
    window_ = std::make_shared<Window>(QString::fromStdString(ros_node_name));

    add_slider_ = ros_node_->getNode().advertiseService("add_slider", &RtGuiServer::addSlider, this);

    QObject::connect(this,        SIGNAL(addSlider(const QString&, const QString&, const double&, const double&)),
                     window_.get(), SLOT(addSlider(const QString&, const QString&, const double&, const double&)));

    return app_->exec();
  }

signals:
  void addSlider(const QString& group_name, const QString& data_name, const double& min, const double& max);

private:


  RtGuiServer()
  {
  }

  //~RtGuiServer()
  //{
  //}

  RtGuiServer(const RtGuiServer&)= delete;
  RtGuiServer& operator=(const RtGuiServer&)= delete;

  std::shared_ptr<Window> window_;
  std::shared_ptr<QApplication> app_;
  std::unique_ptr<RosNode> ros_node_;

  //std::vector<std::pair<double*,double*> > slider_values_;

  ros::ServiceServer add_slider_;
};


} // namespace


#endif
