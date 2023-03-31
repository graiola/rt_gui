#include "rt_gui_ros/rt_gui_client.h"

using namespace rt_gui;

int main(int argc, char* argv[])
{
  double Fz = 0.0;
  bool init = false;
  std::string map_path = "/tmp/map.map";
  std::string server_name, client_name;
  if(argc == 3)
  {
    server_name = argv[1];
    client_name = argv[2];
    init = RtGuiClient::getIstance().init(server_name,client_name); // With namespace
  }
  else
    init = RtGuiClient::getIstance().init(); // Without namespace, use the default rt_gui namespace

  if(init)
  {
    RtGuiClient::getIstance().addDouble(std::string("forces"),std::string("Fz"),-10.5,10.5,&Fz);
    RtGuiClient::getIstance().addText(std::string("exploration"),std::string("map_path"),&map_path);
  }

  // Remove a widget example:
  //RtGuiClient::getIstance().remove(std::string("controllers"),std::string("steps"));

  // Async example: update the data as soon as we receive the new data
  //RtGuiClient::getIstance().addInt(std::string("controllers"),std::string("steps"),0,10,&steps,false);

  while(ros::ok() && RtGuiClient::getIstance().isInitialized())
  {
    RtGuiClient::getIstance().sync();

    ROS_INFO_STREAM("Fz: " << Fz << " map_path: " << map_path);

    ros::Duration(0.1).sleep();
  }
  return 0;
}
