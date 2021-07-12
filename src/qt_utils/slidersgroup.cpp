#include <QtWidgets>

#include "slidersgroup.h"

SlidersGroup::SlidersGroup(const QString &title, const double& min, const double& max,
                           QWidget *parent)
    : QGroupBox(title, parent)
{
    slider_ = new QwtSlider(Qt::Horizontal);
    slider_->setFocusPolicy(Qt::StrongFocus);
    slider_->setScale(min,max);
    slider_->setScalePosition(QwtSlider::ScalePosition::LeadingScale);

    current_ = new QLineEdit();
    QDoubleValidator* validator = new QDoubleValidator(min,max,10,current_);
    validator->setNotation(QDoubleValidator::StandardNotation);
    current_->setValidator(validator);

    connect(slider_, SIGNAL(valueChanged(double)), this, SLOT(setValue(double)));
    connect(current_, SIGNAL(textChanged(QString)), this, SLOT(setValue(QString)));

    QBoxLayout *slidersLayout = new QBoxLayout(QBoxLayout::LeftToRight);
    slidersLayout->addWidget(slider_);
    slidersLayout->addWidget(current_);
    setLayout(slidersLayout);
}

void SlidersGroup::setValue(double value)
{
  value_ = value;
  current_->setText(QString::number(value_));
}

void SlidersGroup::setValue(QString value)
{
  value_ = value.toDouble();
  slider_->setValue(value_);
}
