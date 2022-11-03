#include <rt_gui/qt_utils/label.h>
#include <QStyle>
#include <QTimer>

Label::Label(const QString &client_name, const QString &group_name, const QString &data_name, const QString &placeholder, QWidget *parent)
  : QGroupBox()
{
  group_name_ = group_name;
  data_name_  = data_name;
  client_name_= client_name;

  setObjectName(data_name);

  title_ = new QLabel(data_name);
  QFont font = title_->font();
  font.setWeight(QFont::Bold);
  title_->setFont(font);
  label_ = new QLabel(placeholder);
  label_->setText(placeholder);

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::TopToBottom);
  widgets_layout->addWidget(title_);
  widgets_layout->addWidget(label_);
  setLayout(widgets_layout);

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SIGNAL(updateValue()));
  timer->start(1000);
}

void Label::setValue(QString value)
{
  label_->setText(value);
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

QString Label::getValue()
{
  return value_;
}
