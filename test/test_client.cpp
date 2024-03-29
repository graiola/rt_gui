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

int main(int argc, char* argv[])
{

  double Fx = 2.0;
  bool Filter_on = false;
  bool Controller_on = true;
  int steps = 1;
  std::string status = "Idle";

  std::vector<std::string> controller_list;
  controller_list.push_back("impedance");
  controller_list.push_back("admittance");
  controller_list.push_back("torque");
  std::string controller_type = "torque";

  std::vector<std::string> options_list;
  options_list.push_back("option1");
  options_list.push_back("option2");
  options_list.push_back("option3");
  std::vector<bool*> options_values;
  bool option1 = true;
  bool option2 = true;
  bool option3 = false;
  options_values.push_back(&option1);
  options_values.push_back(&option2);
  options_values.push_back(&option3);


  std::vector<double> velocities(3);

  std::string server_name, client_name;
  if(argc == 3)
  {
    server_name = argv[1];
    client_name = argv[2];
    RtGuiClient::getIstance().init(server_name,client_name); // With namespace
  }
  else
    RtGuiClient::getIstance().init(); // Without namespace, use the default rt_gui namespace

  RtGuiClient::getIstance().addDouble(std::string("forces"),std::string("Fx"),-10.5,10.5,&Fx);
  RtGuiClient::getIstance().addDouble(std::string("forces"),std::string("Fy"),-10.5,10.5,0.0,&setForceY);
  RtGuiClient::getIstance().addDouble(std::string("velocities"),std::string("V"),-100,100,&velocities);
  RtGuiClient::getIstance().addBool(std::string("velocities"),std::string("Filter"),&Filter_on);
  RtGuiClient::getIstance().addList(std::string("controllers"),std::string("type"),controller_list,&controller_type);
  RtGuiClient::getIstance().addBool(std::string("controllers"),std::string("running"),&Controller_on);
  RtGuiClient::getIstance().addTrigger(std::string("controllers"),std::string("stop"),&stopController);
  RtGuiClient::getIstance().addInt(std::string("controllers"),std::string("steps"),0,10,&steps);
  RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status1"),&status);
  RtGuiClient::getIstance().addCheckList(std::string("controllers"),std::string("options"),options_list,options_values);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status2"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status3"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status4"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status5"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status6"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status7"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status8"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status9"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status10"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status11"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status12"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status13"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status14"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status15"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status16"),&status);
  //RtGuiClient::getIstance().addLabel(std::string("controllers"),std::string("status17"),&status);

  // Remove a widget example:
  //RtGuiClient::getIstance().remove(std::string("controllers"),std::string("steps"));

  // Async example: update the data as soon as we receive the new data
  //RtGuiClient::getIstance().addInt(std::string("controllers"),std::string("steps"),0,10,&steps,false);

  long long cnt = 0;

  while(ros::ok())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM(" Fx: " << Fx  <<
                    " Vx: "<<velocities[0] << " Vy: "<< velocities[1] << " Vz: "<< velocities[2] <<
                    " Filter_on: "<<(Filter_on ? "true" : "false") << " Controller: " << controller_type <<
                    " Controller_on: " <<(Controller_on ? "true" : "false") <<
                    " Steps: "   << steps  <<
                    " Status: "  << status <<
                    " Option1: " << (option1 ? "true" : "false") << " "
                    " Option2: " << (option2 ? "true" : "false") << " "
                    " Option3: " << (option3 ? "true" : "false") << " ");

    if((cnt%100) == 0)
    {
      if(status == "Idle")
        status = "Running";
      else
        status = "Idle";
    }
    cnt++;

    ros::Duration(0.1).sleep();
  }

  return 0;
}
