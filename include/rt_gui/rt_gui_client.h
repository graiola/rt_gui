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

  void addSlider(const std::string& group_name, const std::string& data_name, double& data, const double& min, const double& max)
  {
    // Call the ros service
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
    window_->createTabs();
    window_->show();
    stopped_ = false;
    ros_spinner_->start();
    //loop_thread_.reset(new std::thread(&RtGui::loop,this));
    //loop();
  }

  void stop()
  {
    stopped_ = true;
    ros_spinner_->stop();
  }

private:

  void loop()
  {
    while(!stopped_)
    {
      //QApplication::instance()->processEvents();
      std::this_thread::sleep_for( std::chrono::milliseconds(4) ); //ms
    }
  }

  RtGuiClient()
  {
    stopped_ = false;
    ros_spinner_.reset(new ros::AsyncSpinner(1));
    sync_ = ros_nh_ptr_->advertiseService("sync", &RtGui::sync, this);
  }

  //run_in_gui_thread(new RunEventImpl([](){
  //        QMainWindow* window=new QMainWindow();
  //        window->show();
  //    }));

  ~RtGuiClient()
  {
  }

  RtGuiClient(const RtGuiClient&)= delete;
  RtGuiClient& operator=(const RtGuiClient&)= delete;

  std::unique_ptr<RosNode> ros_node_;

  std::vector<std::pair<double*,double*> > slider_values_;
  std::atomic<bool> stopped_;

  std::shared_ptr<ros::AsyncSpinner> ros_spinner_;
  ros::ServiceServer sync_;

};


} // namespace


#endif
