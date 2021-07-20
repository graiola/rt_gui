#include "qt_utils/double_slider.h"

DoubleSlider::DoubleSlider(const QString &group_name, const QString &data_name, const double& min, const double& max, const double& init,
               QWidget *parent)
  : QGroupBox(data_name, parent)
{
  group_name_ = group_name;
  data_name_  = data_name;

  setObjectName(data_name);

  slider_ = new QwtSlider(Qt::Horizontal);
  slider_->setFocusPolicy(Qt::StrongFocus);
  slider_->setScale(min,max);
  slider_->setScalePosition(QwtSlider::ScalePosition::LeadingScale);
  slider_->setValue(init);

  current_ = new QLineEdit();
  QDoubleValidator* validator = new QDoubleValidator(min,max,10,current_);
  validator->setNotation(QDoubleValidator::StandardNotation);
  current_->setValidator(validator);
  current_->setText(QString::number(init));

  max_ = new QLabel(QString::number(max));
  min_ = new QLabel(QString::number(min));

  connect(slider_, SIGNAL(valueChanged(double)), this, SLOT(setValue(double)));
  connect(current_, SIGNAL(textChanged(QString)), this, SLOT(setValue(QString)));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(min_);
  widgets_layout->addWidget(slider_);
  widgets_layout->addWidget(max_);
  widgets_layout->addWidget(current_);
  setLayout(widgets_layout);
}

double DoubleSlider::getValue()
{
  return value_;
}

const QString &DoubleSlider::getDataName() const
{
  return data_name_;
}

const QString &DoubleSlider::getGroupName() const
{
  return group_name_;
}

void DoubleSlider::setValue(double value)
{
  value_ = value;
  current_->setText(QString::number(value_));
  emit valueChanged(value_);
}

void DoubleSlider::setValue(QString value)
{
  value_ = value.toDouble();
  slider_->setValue(value_);
  emit valueChanged(value_);
}