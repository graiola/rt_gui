#ifndef RT_GUI_QT_UTILS_BUTTON_H
#define RT_GUI_QT_UTILS_BUTTON_H

#include <QtWidgets>
#include <QPushButton>

QT_BEGIN_NAMESPACE
class QPushButton;
QT_END_NAMESPACE

class Button : public QGroupBox
{
  Q_OBJECT

public:
  Button(const QString &group_name, const QString &data_name, QWidget *parent = nullptr);

  const QString& getDataName() const;
  const QString& getGroupName() const;

signals:
  void valueChanged();

public slots:
  void setValue();

private:
  QPushButton*      button_;
  QString           data_name_;
  QString           group_name_;
};

#endif
