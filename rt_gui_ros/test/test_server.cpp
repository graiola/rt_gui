#include "rt_gui_ros/rt_gui_server.h"

using namespace rt_gui;

int main(int argc, char *argv[])
{
  return RtGuiServer::getIstance().run(argc,argv);
}
