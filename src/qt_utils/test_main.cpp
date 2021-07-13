#include <QApplication>
#include <QString>

#include "window.h"
#include "rt_gui/rt_gui.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Window window;
  window.addSlider(QString("forces"),QString("Fx"),-10.5,10.5);
  window.addSlider(QString("forces"),QString("Fy"),-10.5,10.5);
  window.addSlider(QString("velocities"),QString("Vx"),-12.5,16.5);
  window.createTabs();
  window.show();
  return app.exec();
}
