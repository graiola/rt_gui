#ifndef RT_GUI_QT_UTILS_DOUBLE_SLIDERS_H
#define RT_GUI_QT_UTILS_DOUBLE_SLIDERS_H

#include <QtWidgets>
#include <QGroupBox>
#include <qwt/qwt_slider.h>

QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
QT_END_NAMESPACE

class DoubleSlider : public QGroupBox
{
  Q_OBJECT

public:
  DoubleSlider(const QString &client_name, const QString &group_name, const QString &data_name, const double& min, const double& max, const double& init,
         QWidget *parent = nullptr);

  double getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

  friend QDataStream &operator<<(QDataStream &out, DoubleSlider* widget);

signals:
  void valueChanged(double value);

public slots:
  void setValue();
  void setValue(double value);

private:
  QwtSlider*      slider_;
  QLineEdit*      current_;
  double          value_;
  QLabel*         max_;
  QLabel*         min_;

  QString         data_name_;
  QString         group_name_;
  QString         client_name_;
};

#endif
