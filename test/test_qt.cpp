#include <QApplication>
#include <QString>

#include "qt_utils/window.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Window window(QString("rt_gui"));
  window.addSlider(QString("forces"),QString("Fx"),-10.5,10.5,0.0);
  window.addSlider(QString("forces"),QString("Fy"),-10.5,10.5,0.0);
  window.addSlider(QString("velocities"),QString("Vx"),-12.5,16.5,0.0);
  return app.exec();
}
