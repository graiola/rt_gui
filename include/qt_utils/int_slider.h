#ifndef QT_UTILS_INT_SLIDERS_H
#define QT_UTILS_INT_SLIDERS_H

#include <QtWidgets>
#include <QGroupBox>
#include <QSlider>

QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
class QSlider;
QT_END_NAMESPACE

class IntSlider : public QGroupBox
{
  Q_OBJECT

public:
  IntSlider(const QString &group_name, const QString &data_name, const int& min, const int& max, const int& init,
         QWidget *parent = 0);

  int getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;

signals:
  void valueChanged(int value);

public slots:
  void setValue();
  void setValue(int value);

private:
  QSlider*        slider_;
  QLineEdit*      current_;
  double          value_;
  QLabel*         max_;
  QLabel*         min_;

  QString         data_name_;
  QString         group_name_;
};

#endif
