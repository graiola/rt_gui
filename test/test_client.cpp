#include "rt_gui/rt_gui_client.h"

using namespace rt_gui;

int main(int /*argc*/, char*[] /*argv[]*/)
{

  double Fx = 2.0, Vx = 0.0, Vy = -50.0;

  RtGuiClient::getIstance().addSlider(std::string("forces"),std::string("Fx"),-10.5,10.5,&Fx);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vx"),-10.5,10.5,&Vx);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vy"),-100,10.5,&Vy);

  ros::Rate r(100);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM("Fx: " << Fx <<" "<< "Vx: "<<Vx << " Vy: "<< Vy);

    r.sleep();
  }
  return 0;
}
