#include "rt_gui/rt_gui.h"

using namespace rt_gui;

int main(int argc, char *argv[])
{

  double Fx = 0;
  double Fy = 0;
  double Vx = 0;
  RtGui::getGui().addSlider("forces","Fx",Fx,-10.5,10.5);
  RtGui::getGui().addSlider("forces","Fy",Fy,-10.5,10.5);
  RtGui::getGui().addSlider("velocities","Vx",Vx,-12.5,16.5);

  RtGui::getGui().run();

  getchar();

  //ros::Rate r(10);

  //while(ros::ok())
  //{
    //RtGui::getGui().sync();
    //ROS_INFO_STREAM("++++++++++++++++++++++++");
    //ROS_INFO_STREAM("Fx: " << Fx << " Fy: " << Fy << " Vx: " << Vx);

  //  ros::spinOnce();
  //  r.sleep();
  //}

  RtGui::getGui().stop();

  return 0;

}
