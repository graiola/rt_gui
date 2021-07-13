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

  template <typename data_t>
  void addSlider(const std::string& group_name, const std::string& data_name, const data_t& data, const double& min, const double& max)
  {
    window_->addSlider(QString::fromStdString(group_name),QString::fromStdString(data_name),min,max);
  }

  void sync()
  {

  }

  void sync(const std::string& data_name)
  {

  }

  int run()
  {
    if(init_)
    {
      ros::Rate r(100);
      window_->createTabs();
      window_->show();
      while(ros::ok())
      {
        app_->processEvents();
        ros::spinOnce();
        r.sleep();
      }
    }
    else
      return 1;

    return 0;
  }

private:

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

    app_ = std::make_shared<QApplication>(argc_, argv_);
    window_ = std::make_shared<Window>();

  }

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

};


} // namespace


#endif
