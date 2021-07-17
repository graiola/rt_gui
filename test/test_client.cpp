#include "rt_gui/rt_gui_client.h"

using namespace rt_gui;

int main(int /*argc*/, char*[] /*argv[]*/)
{

  double Fx = 2.0, Vx = 0.0, Vy = -50.0;
  bool Filter_on = false;

  std::vector<std::string> controller_list;
  controller_list.push_back("impedance");
  controller_list.push_back("admittance");
  controller_list.push_back("torque");

  std::string controller_type = "torque";

  RtGuiClient::getIstance().addSlider(std::string("forces"),std::string("Fx"),-10.5,10.5,&Fx);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vx"),-10.5,10.5,&Vx);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vy"),-100,10.5,&Vy);
  RtGuiClient::getIstance().addRadioButton(std::string("velocities"),std::string("Filter"),&Filter_on);
  RtGuiClient::getIstance().addComboBox(std::string("controllers"),std::string("type"),controller_list,&controller_type);

  ros::Rate r(100);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM("Fx: " << Fx <<" "<< "Vx: "<<Vx << " Vy: "<< Vy << " Filter_on: "<<(Filter_on ? "true" : "false") << " Controller: " << controller_type);

    r.sleep();
  }
  return 0;
}
