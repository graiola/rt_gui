#ifndef RT_GUI_CORE_QT_UTILS_LABEL_H
#define RT_GUI_CORE_QT_UTILS_LABEL_H

#include <QtWidgets>
#include <QLabel>

QT_BEGIN_NAMESPACE
class QLabel;
QT_END_NAMESPACE

class Label : public QGroupBox
{
  Q_OBJECT

public:
  Label(const QString &client_name, const QString &group_name, const QString &data_name, const QString &placeholder, QWidget *parent = nullptr);

  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

  QString getValue();

signals:
  void updateValue();

public slots:
  void setValue(QString value);

private:
  QLabel*           title_;
  QLabel*           label_;
  QString           data_name_;
  QString           group_name_;
  QString           client_name_;
  QString           value_;
};

#endif
