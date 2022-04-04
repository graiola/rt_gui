#include <rt_gui/qt_utils/button.h>

Button::Button(const QString &group_name, const QString &data_name, QWidget *parent)
  : QGroupBox()
{
  group_name_ = group_name;
  data_name_  = data_name;

  setObjectName(data_name);

  button_ = new QPushButton(data_name);

  connect(button_,  SIGNAL(clicked()), this, SLOT(setValue()));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(button_);
  setLayout(widgets_layout);
}

void Button::setValue()
{
  emit valueChanged();
}

const QString &Button::getDataName() const
{
  return data_name_;
}

const QString &Button::getGroupName() const
{
  return group_name_;
}
