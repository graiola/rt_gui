#include "rt_gui/rt_gui.h"

using namespace rt_gui;

int main(int argc, char *argv[])
{

  double Fx = 0;
  double Fy = 0;
  double Vx = 0;
  RtGui::getGui().addSlider("forces","Fx",Fx,-10.5,10.5);
  RtGui::getGui().addSlider("forces","Fy",Fx,-10.5,10.5);
  RtGui::getGui().addSlider("velocities","Vx",Vx,-12.5,16.5);

  return RtGui::getGui().run();
}
