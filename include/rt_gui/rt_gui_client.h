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
#include <rt_gui/sync.h>

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

  bool sync(sync::Request &req,
            sync::Response &res)
  {
    // FIXME no thread safe
    //for(unsigned int i=0;i<slider_values_.size();i++)
    //  *slider_values_[i].first = *slider_values_[i].second;

  }

  void run()
  {
    //window_->createTabs();
    //window_->show();
    //stopped_ = false;
    //ros_spinner_->start();
    //loop_thread_.reset(new std::thread(&RtGui::loop,this));
    //loop();
  }

  void stop()
  {

  }

private:

  void loop()
  {
    //while(!stopped_)
    //{
    //  //QApplication::instance()->processEvents();
    //  std::this_thread::sleep_for( std::chrono::milliseconds(4) ); //ms
    //}
  }

  RtGuiClient()
  {
    std::string ros_node_name = RT_GUI_CLIENT_NAME;
    ros_node_.reset(new RosNode(ros_node_name));
    add_slider_ = ros_node_->getNode().serviceClient<rt_gui::addSlider>("/"RT_GUI_SERVER_NAME"/add_slider");
  }


  ~RtGuiClient()
  {
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;
  ros::ServiceServer sync_;

  ros::ServiceClient add_slider_;
};


} // namespace


#endif
