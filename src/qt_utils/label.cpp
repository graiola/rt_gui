#include <rt_gui/qt_utils/label.h>
#include <QStyle>

Label::Label(const QString &client_name, const QString &group_name, const QString &data_name, const QString &placeholder, QWidget *parent)
  : QGroupBox()
{
  group_name_ = group_name;
  data_name_  = data_name;
  client_name_= client_name;

  setObjectName(data_name);

  title_ = new QLabel(placeholder);

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::TopToBottom);
  widgets_layout->addWidget(title_);
  setLayout(widgets_layout);
}

void Label::setValue(QString value)
{
  title_->setText(value);
}

QString Label::getValue()
{
  return value_;
}

const QString &Label::getDataName() const
{
  return data_name_;
}

const QString &Label::getGroupName() const
{
  return group_name_;
}

const QString &Label::getClientName() const
{
  return client_name_;
}
