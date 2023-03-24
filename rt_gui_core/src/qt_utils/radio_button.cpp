#include <rt_gui_core/qt_utils/radio_button.h>

RadioButton::RadioButton(const QString &client_name, const QString &group_name, const QString &data_name, const bool& init, QWidget *parent)
  : QGroupBox(data_name, parent)
{
  group_name_  = group_name;
  data_name_   = data_name;
  client_name_ = client_name;

  setObjectName(data_name);

  true_ = new QRadioButton("True");
  false_ = new QRadioButton("False");

  if(init)
    true_->setChecked(true);
  else
    false_->setChecked(true);

  value_ = init;

  connect(true_, SIGNAL(clicked()), this, SLOT(setValueTrue()));
  connect(false_, SIGNAL(clicked()), this, SLOT(setValueFalse()));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(true_);
  widgets_layout->addWidget(false_);
  setLayout(widgets_layout);
}

void RadioButton::setValueTrue()
{
  value_ = true;
  emit valueChanged(value_);
}

void RadioButton::setValueFalse()
{
  value_ = false;
  emit valueChanged(value_);
}

bool RadioButton::getValue()
{
  return value_;
}

const QString &RadioButton::getDataName() const
{
  return data_name_;
}

const QString &RadioButton::getGroupName() const
{
  return group_name_;
}

const QString &RadioButton::getClientName() const
{
  return client_name_;
}
