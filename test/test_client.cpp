#include "rt_gui/rt_gui_client.h"

using namespace rt_gui;

int main(int argc, char *argv[])
{

  RtGuiClient::getIstance().addSlider(std::string("forces"),std::string("Fx"),-10.5,10.5);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vx"),-10.5,10.5);
  RtGuiClient::getIstance().addSlider(std::string("velocities"),std::string("Vy"),-100,10.5);



  return RtGuiClient::getIstance().run();
}
