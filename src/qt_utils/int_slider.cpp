#include <rt_gui/qt_utils/int_slider.h>

IntSlider::IntSlider(const QString &group_name, const QString &data_name, const int& min, const int& max, const int& init,
               QWidget *parent)
  : QGroupBox(data_name, parent)
{
  group_name_ = group_name;
  data_name_  = data_name;

  setObjectName(data_name);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setFocusPolicy(Qt::StrongFocus);
  slider_->setMaximum(max);
  slider_->setMinimum(min);
  slider_->setValue(init);

  current_ = new QLineEdit();
  QIntValidator* validator = new QIntValidator(min,max,current_);
  current_->setValidator(validator);
  current_->setText(QString::number(init));

  max_ = new QLabel(QString::number(max));
  min_ = new QLabel(QString::number(min));

  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
  connect(current_, SIGNAL(returnPressed()), this, SLOT(setValue()));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(min_);
  widgets_layout->addWidget(slider_);
  widgets_layout->addWidget(max_);
  widgets_layout->addWidget(current_);
  setLayout(widgets_layout);
}

int IntSlider::getValue()
{
  return value_;
}

const QString &IntSlider::getDataName() const
{
  return data_name_;
}

const QString &IntSlider::getGroupName() const
{
  return group_name_;
}

void IntSlider::setValue(int value)
{
  value_ = value;
  current_->setText(QString::number(value_));
  emit valueChanged(value_);
}

void IntSlider::setValue()
{
  value_ = current_->text().toInt();
  slider_->setValue(value_);
  emit valueChanged(value_);
}
