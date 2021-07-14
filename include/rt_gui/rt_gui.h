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

#ifndef RT_GUI_RT_GUI_H
#define RT_GUI_RT_GUI_H

#include <qt_utils/window.h>
#include <ros/ros.h>

#include <memory>
#include <atomic>
#include <thread>

namespace rt_gui
{

class RtGui
{
public:

  static RtGui& getGui()
  {
    static RtGui gui;
    return gui;
  }

  void addSlider(const std::string& group_name, const std::string& data_name, double& data, const double& min, const double& max)
  {
    std::pair<double*,double*> p;
    p.second = window_->addSlider(QString::fromStdString(group_name),QString::fromStdString(data_name),min,max);
    p.first = &data;
    slider_values_.push_back(p);
  }

  void sync()
  {
    // FIXME no thread safe
    for(unsigned int i=0;i<slider_values_.size();i++)
      *slider_values_[i].first = *slider_values_[i].second;
  }

  void run()
  {
    window_->createTabs();
    window_->show();
    stopped_ = false;
    loop_thread_.reset(new std::thread(&RtGui::loop,this));
    //loop();
  }

  void stop()
  {
    stopped_ = true;
    loop_thread_->join();
    //ros_spinner_->stop();
  }

private:

  void loop()
  {
    while(!stopped_)
    {
      app_->processEvents();

      //QApplication::instance()->processEvents();
      std::this_thread::sleep_for( std::chrono::milliseconds(4) ); //ms
    }
  }

  RtGui()
  {

    std::string ros_node_name = "rt_gui";
    argc_ = 1;
    argv_[0] = strdup(ros_node_name.c_str());
    argv_[1] = 0;
    ros::init(argc_, argv_, ros_node_name,ros::init_options::NoSigintHandler);

    if(ros::master::check())
      ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
    else
    {
      std::string err("roscore not found... Did you start the server?");
      throw std::runtime_error(err);
    }

    init_ = true;
    stopped_ = false;

    app_ = std::make_shared<QApplication>(argc_, argv_);
    window_ = std::make_shared<Window>();

    //ros_spinner_.reset(new ros::AsyncSpinner(1));
    //ros_spinner_->start();

  }

  //run_in_gui_thread(new RunEventImpl([](){
  //        QMainWindow* window=new QMainWindow();
  //        window->show();
  //    }));

  ~RtGui()
  {
    free (argv_[0]);
  }



  RtGui(const RtGui&)= delete;
  RtGui& operator=(const RtGui&)= delete;

  std::shared_ptr<Window> window_;
  std::shared_ptr<QApplication> app_;

  ros::NodeHandle* ros_nh_ptr_;
  bool init_;

  int argc_;
  char* argv_[2];

  std::vector<std::pair<double*,double*> > slider_values_;
  std::shared_ptr<std::thread> loop_thread_;

  std::atomic<bool> stopped_;
  //std::shared_ptr<ros::AsyncSpinner> ros_spinner_;

};


} // namespace


#endif
