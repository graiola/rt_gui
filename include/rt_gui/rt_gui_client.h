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

  typedef std::pair<std::string,std::string> sliders_buffer_key_t;
  typedef std::pair<double*,double*> sliders_buffer_value_t;
  typedef std::map<sliders_buffer_key_t, sliders_buffer_value_t> sliders_buffer_t;

  static RtGuiClient& getIstance()
  {
    static RtGuiClient istance;
    return istance;
  }

  void addSlider(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
  {

    assert(data_ptr);
    rt_gui::addSlider srv;

    srv.request.min = min;
    srv.request.max = max;
    srv.request.init = *data_ptr;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;

    if(add_slider_.exists())
    {
      add_slider_.call(srv);
      if(srv.response.resp == false)
        throw std::runtime_error("RtGuiServer::addSlider::resp is false!");
      else
        sliders_buffer_[sliders_buffer_key_t(group_name,data_name)] = sliders_buffer_value_t(data_ptr,new double(*data_ptr));
    }
    else
    {
      throw std::runtime_error("RtGuiServer::addSlider service is not available!");
    }
  }

  bool updateClient(updateClient::Request &req,
                    updateClient::Response &res)
  {
     ROS_DEBUG_STREAM("updateClient: " << req.group_name << " " << req.data_name << " " << req.value << std::endl);
     sync_mtx_.lock();
     *sliders_buffer_[sliders_buffer_key_t(req.group_name,req.data_name)].second = req.value;
     sync_mtx_.unlock();
     // FIXME add a proper error handling
     res.resp = true;

     return res.resp;
  }

  //int run() // FIXME to be threaded away.........
  //{
  //  ros::Rate r(50);
  //  while(ros::ok())
  //  {
  //    r.sleep();
  //  }
  //  return 0;
  //}

  void sync()
  {
    if (sync_mtx_.try_lock())
    {
      for(auto tmp_map : sliders_buffer_)
        *tmp_map.second.first = *tmp_map.second.second;
      sync_mtx_.unlock();
    }
  }

private:

  RtGuiClient()
  {
    std::string ros_node_name = RT_GUI_CLIENT_NAME;
    ros_node_.reset(new RosNode(ros_node_name));

    add_slider_ = ros_node_->getNode().serviceClient<rt_gui::addSlider>("/" RT_GUI_SERVER_NAME "/add_slider");
    update_client_ = ros_node_->getNode().advertiseService("update_client", &RtGuiClient::updateClient, this);

  }

  ~RtGuiClient()
  {
    for(auto tmp_map : sliders_buffer_)
    {
      //if(tmp_map.second.first!=nullptr)
      //  delete tmp_map.second.first;
      if(tmp_map.second.second!=nullptr)
        delete tmp_map.second.second;
    }
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;

  ros::ServiceServer update_client_;
  ros::ServiceClient add_slider_;

  sliders_buffer_t sliders_buffer_;

  std::mutex sync_mtx_;
};


} // namespace


#endif
