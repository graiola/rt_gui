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

#include <rt_gui/common.h>

#include <qt_utils/window.h>
#include <rt_gui/addSlider.h>
#include <rt_gui/updateClient.h>

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

  void addSlider(const std::string& group_name, const std::string& data_name, const double& min, const double& max)
  {
    rt_gui::addSlider srv;

    srv.request.min = min;
    srv.request.max = max;
    srv.request.init = 0; // FIXME
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;

    if(add_slider_.exists())
    {
      add_slider_.call(srv);
      if(srv.response.resp == false)
        throw std::runtime_error("RtGuiServer::addSlider::resp is false!");
    }
    else
    {
      throw std::runtime_error("RtGuiServer::addSlider service is not available!");
    }
  }

  bool updateClient(updateClient::Request &req,
                    updateClient::Response &res)
  {
     ROS_INFO_STREAM("updateClient: " << req.group_name << " " << req.data_name << " " << req.value << std::endl);

     // FIXME add a proper error handling
     res.resp = true;

     return res.resp;
  }

  int run() // FIXME to be threaded away.........
  {
    ros::Rate r(50);
    while(ros::ok())
    {
      r.sleep();
    }
    return 0;
  }

private:

  RtGuiClient()
  {
    std::string ros_node_name = RT_GUI_CLIENT_NAME;
    ros_node_.reset(new RosNode(ros_node_name));

    add_slider_ = ros_node_->getNode().serviceClient<rt_gui::addSlider>("/"RT_GUI_SERVER_NAME"/add_slider");
    update_client_ = ros_node_->getNode().advertiseService("update_client", &RtGuiClient::updateClient, this);

  }

  //~RtGuiClient()
  //{
  //}

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;

  ros::ServiceServer update_client_;
  ros::ServiceClient add_slider_;
};


} // namespace


#endif
