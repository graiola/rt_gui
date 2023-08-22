#ifndef RT_GUI_QT_UTILS_CHECK_BOX_H
#define RT_GUI_QT_UTILS_CHECK_BOX_H

#include <QtWidgets>
#include <QGroupBox>
#include <QCheckBox>
#include <QVector>

QT_BEGIN_NAMESPACE
class QListWidget;
class QStringList;
QT_END_NAMESPACE

class CheckBox : public QGroupBox
{
  Q_OBJECT

public:
  CheckBox(const QString &client_name, const QString &group_name, const QString &data_name, const QStringList& list, const QVector<bool>& init, QWidget *parent = nullptr);

  QVector<bool> getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;
  const QString& getClientName() const;

signals:
  void valueChanged(QVector<bool> value);

public slots:
  void setValue(QListWidgetItem *item);

private:
  QListWidget*    widget_list_;

  QString         data_name_;
  QString         group_name_;
  QString         client_name_;

  QVector<bool>   value_;
};

#endif
