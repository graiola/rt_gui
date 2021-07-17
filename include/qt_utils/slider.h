#ifndef QT_UTILS_SLIDERS_H
#define QT_UTILS_SLIDERS_H

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
  Slider(const QString &group_name, const QString &data_name, const double& min, const double& max, const double& init,
         QWidget *parent = 0);

  double getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;

signals:
  void valueChanged(double value);

public slots:
  void setValue(QString value);
  void setValue(double value);

private:
  QwtSlider*      slider_;
  QLineEdit*      current_;
  double          value_;
  QLabel*         max_;
  QLabel*         min_;

  QString         data_name_;
  QString         group_name_;
};

#endif
