#include <QApplication>
#include <QString>

#include "qt_utils/window.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  Window window(QString("rt_gui"));

  WidgetsGroup group1;
  QLabel* label11 = new QLabel("label11");
  QLabel* label12 = new QLabel("label12");
  QLabel* label13 = new QLabel("label13");
  QLabel* label14 = new QLabel("label14");
  QLabel* label15 = new QLabel("label15");
  QLabel* label16 = new QLabel("label16");
  QLabel* label17 = new QLabel("label17");
  QLabel* label18 = new QLabel("label18");
  QLabel* label19 = new QLabel("label19");

  group1.add(label11);
  group1.add(label12);
  group1.add(label13);
  group1.add(label14);
  group1.add(label15);
  group1.add(label16);
  group1.add(label17);
  group1.add(label18);
  group1.add(label19);

  WidgetsGroup group2;
  QLabel* label21 = new QLabel("label21");
  QLabel* label22 = new QLabel("label22");
  QLabel* label23 = new QLabel("label23");
  QLabel* label24 = new QLabel("label24");
  QLabel* label25 = new QLabel("label25");
  QLabel* label26 = new QLabel("label26");
  QLabel* label27 = new QLabel("label27");
  QLabel* label28 = new QLabel("label28");
  QLabel* label29 = new QLabel("label29");

  group2.add(label21);
  group2.add(label22);
  group2.add(label23);
  group2.add(label24);
  group2.add(label25);
  group2.add(label26);
  group2.add(label27);
  group2.add(label28);
  group2.add(label29);

  QTabWidget* tab = new QTabWidget;

  tab->addTab(&group1,"group1");
  tab->addTab(&group2,"group2");

  QScrollArea* scroll = new QScrollArea;
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setWidgetResizable(true);
  scroll->setVisible(true);
  scroll->setWidget(tab);

  return app.exec();
}
