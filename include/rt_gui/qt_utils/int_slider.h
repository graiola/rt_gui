#ifndef RT_GUI_QT_UTILS_INT_SLIDERS_H
#define RT_GUI_QT_UTILS_INT_SLIDERS_H

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
  IntSlider(const QString &client_name, const QString &group_name, const QString &data_name, const int& min, const int& max, const int& init,
         QWidget *parent = nullptr);

  int getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

  friend QDataStream &operator<<(QDataStream &out, IntSlider* widget);

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
  QString         client_name_;
};

#endif
