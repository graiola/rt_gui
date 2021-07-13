#ifndef QT_UTILS_SLIDERSGROUP_H
#define QT_UTILS_SLIDERSGROUP_H

#include <QtWidgets>
#include <QGroupBox>
#include <QSlider>
#include <qwt/qwt_slider.h>

QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
QT_END_NAMESPACE

class Slider : public QGroupBox
{
  Q_OBJECT

public:
  Slider(const QString &title, const double& min, const double& max,
         QWidget *parent = 0);

signals:
  void valueChanged(int value);

public slots:
  void setValue(QString value);
  void setValue(double value);

private:
  QwtSlider*      slider_;
  QLineEdit*      current_;
  double          value_;
  QLabel*         max_;
  QLabel*         min_;
};

class SlidersGroup : public QGroupBox
{
  Q_OBJECT

public:
  SlidersGroup(const QString &title,
               QWidget *parent = 0);

public:
  void addSlider(Slider* slider);

private:
  QBoxLayout* sliders_layout_;
};

#endif
