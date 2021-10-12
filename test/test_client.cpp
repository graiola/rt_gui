#include "rt_gui/rt_gui_client.h"

using namespace rt_gui;

bool stopController()
{
  ROS_WARN("Stopping the controller!");
  return true;
}

bool setForceY(const double& Fy)
{
  ROS_WARN_STREAM("Fy:"<<Fy);
  return true;
}

int main(int /*argc*/, char*[] /*argv[]*/)
{

  double Fx = 2.0;
  bool Filter_on = false;
  bool Controller_on = true;
  int steps = 1;

  std::vector<std::string> controller_list;
  controller_list.push_back("impedance");
  controller_list.push_back("admittance");
  controller_list.push_back("torque");
  std::string controller_type = "torque";

  std::vector<double> velocities(3);

  RtGuiClient::getIstance().addDouble(std::string("forces"),std::string("Fx"),-10.5,10.5,&Fx);
  RtGuiClient::getIstance().addDouble(std::string("forces"),std::string("Fy"),-10.5,10.5,0.0,&setForceY);
  RtGuiClient::getIstance().addDouble(std::string("velocities"),std::string("V"),-100,100,&velocities);
  RtGuiClient::getIstance().addBool(std::string("velocities"),std::string("Filter"),&Filter_on);
  RtGuiClient::getIstance().addList(std::string("controllers"),std::string("type"),controller_list,&controller_type);
  RtGuiClient::getIstance().addBool(std::string("controllers"),std::string("status"),&Controller_on);
  RtGuiClient::getIstance().addTrigger(std::string("controllers"),std::string("stop"),&stopController);
  RtGuiClient::getIstance().addInt(std::string("controllers"),std::string("steps"),0,10,&steps);

  // Remove a widget example:
  //RtGuiClient::getIstance().remove(std::string("controllers"),std::string("steps"));

  // Async example: update the data as soon as we receive the new data
  //RtGuiClient::getIstance().addInt(std::string("controllers"),std::string("steps"),0,10,&steps,false);

  ros::Rate r(100);
  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM("Fx: " << Fx <<" "<< "Vx: "<<velocities[0] << " Vy: "<< velocities[1] << " Vz: "<< velocities[2] << " Filter_on: "<<(Filter_on ? "true" : "false") << " Controller: " << controller_type
                    <<" status: " <<(Controller_on ? "true" : "false") << " steps: " << steps );

    r.sleep();
  }
  return 0;
}
