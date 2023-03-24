#ifndef RT_GUI_CORE_QT_UTILS_RADIO_BUTTON_H
#define RT_GUI_CORE_QT_UTILS_RADIO_BUTTON_H

#include <QtWidgets>
#include <QGroupBox>
#include <QRadioButton>

QT_BEGIN_NAMESPACE
class QRadioButton;
QT_END_NAMESPACE

class RadioButton : public QGroupBox
{
  Q_OBJECT

public:
  RadioButton(const QString &client_name, const QString &group_name, const QString &data_name, const bool& init, QWidget *parent = nullptr);

  bool getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

signals:
  void valueChanged(bool value);

public slots:
  void setValueTrue();
  void setValueFalse();

private:
  QRadioButton*      true_;
  QRadioButton*      false_;

  QString         data_name_;
  QString         group_name_;
  QString         client_name_;

  bool value_;
};

#endif
