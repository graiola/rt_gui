#include <QApplication>
#include <QString>

#include "qt_utils/window.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Window window(QString("rt_gui"));

  QStringList list;
  list.push_back("impedance");
  list.push_back("admittance");
  list.push_back("position");

  window.addDoubleSlider(QString("forces"),QString("Fx"),-10.5,10.5,0.0);
  window.addDoubleSlider(QString("forces"),QString("Fy"),-10.5,10.5,0.0);
  window.addDoubleSlider(QString("velocities"),QString("Vx"),-12.5,16.5,0.0);
  window.addRadioButton(QString("velocities"),QString("Filter"),false);
  window.addComboBox(QString("controllers"),QString("types"),list,QString("admittance"));
  window.addButton(QString("main"),QString("stop"));
  window.addIntSlider(QString("main"),QString("steps"),0,5,1);

  //window.removeWidget(QString("velocities"),QString("Filter"));

  return app.exec();
}
