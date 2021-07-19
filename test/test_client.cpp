#include "rt_gui/rt_gui_client.h"

using namespace rt_gui;

bool stopController()
{
  ROS_WARN("Stopping the controller!");
  return true;
}

int main(int /*argc*/, char*[] /*argv[]*/)
{

  double Fx = 2.0;
  bool Filter_on = false;
  bool Controller_on = true;

  std::vector<std::string> controller_list;
  controller_list.push_back("impedance");
  controller_list.push_back("admittance");
  controller_list.push_back("torque");
  std::string controller_type = "torque";

  std::vector<double> velocities(3);


  RtGuiClient::getIstance().addSlider(std::string("forces"),std::string("Fx"),-10.5,10.5,&Fx);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("V"),-100,100,&velocities);
  RtGuiClient::getIstance().addRadioButton(std::string("velocities"),std::string("Filter"),&Filter_on);
  RtGuiClient::getIstance().addComboBox(std::string("controllers"),std::string("type"),controller_list,&controller_type);
  RtGuiClient::getIstance().addRadioButton(std::string("controllers"),std::string("status"),&Controller_on);
  RtGuiClient::getIstance().addButton(std::string("controllers"),std::string("stop"),&stopController);

  ros::Rate r(100);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM("Fx: " << Fx <<" "<< "Vx: "<<velocities[0] << " Vy: "<< velocities[1] << " Vz: "<< velocities[2] << " Filter_on: "<<(Filter_on ? "true" : "false") << " Controller: " << controller_type
                    <<" status: " <<(Controller_on ? "true" : "false") );

    r.sleep();
  }
  return 0;
}
