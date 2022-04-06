#ifndef RT_GUI_QT_UTILS_COMBO_BOX_H
#define RT_GUI_QT_UTILS_COMBO_BOX_H

#include <QtWidgets>
#include <QGroupBox>
#include <QComboBox>

QT_BEGIN_NAMESPACE
class QComboBox;
class QStringList;
QT_END_NAMESPACE

class ComboBox : public QGroupBox
{
  Q_OBJECT

public:
  ComboBox(const QString &group_name, const QString &data_name, const QStringList& list, const QString& init, QWidget *parent = nullptr);

  QString getValue();
  const QString& getDataName() const;
  const QString& getGroupName() const;

signals:
  void valueChanged(QString value);

public slots:
  void setValue(const QString& value);

private:
  QComboBox*      combo_;

  QString         data_name_;
  QString         group_name_;

  QString         value_;
};

#endif
