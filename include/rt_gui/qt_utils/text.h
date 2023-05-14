#ifndef RT_GUI_QT_UTILS_TEXT_H
#define RT_GUI_QT_UTILS_TEXT_H

#include <QtWidgets>
#include <QLineEdit>
#include <QToolButton>
#include <QLabel>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QLabel;
QT_END_NAMESPACE

class Text : public QGroupBox
{
  Q_OBJECT

public:
  Text(const QString &client_name, const QString &group_name, const QString &data_name, const QString &placeholder, QWidget *parent = nullptr);

  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

  QString getValue();

  friend QDataStream &operator<<(QDataStream &out, Text* widget);

signals:
  void valueChanged(QString value);

public slots:
  void setValue();

private:
  QLineEdit*        text_;
  QLabel*           title_;
  QString           data_name_;
  QString           group_name_;
  QString           client_name_;
  QString           value_;
};

#endif
