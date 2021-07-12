#include "slidersgroup.h"
#include "window.h"


Window::Window()
{
    //horizontalSliders = new SlidersGroup(Qt::Horizontal, tr("Horizontal"));
    //verticalSliders = new SlidersGroup(Qt::Vertical, tr("Vertical"));
    //
    //stacked_widget_ = new QStackedWidget;
    //stacked_widget_->addWidget(horizontalSliders);
    //stacked_widget_->addWidget(verticalSliders);
    //
    //createControls(tr("Controls"));

    //connect(horizontalSliders, SIGNAL(valueChanged(int)),
    //        verticalSliders, SLOT(setValue(int)));
    //connect(verticalSliders, SIGNAL(valueChanged(int)),
    //        valueSpinBox, SLOT(setValue(int)));
    //connect(valueSpinBox, SIGNAL(valueChanged(int)),
    //        horizontalSliders, SLOT(setValue(int)));

    stacked_widget_ = new QStackedWidget;
    layout_ = new QHBoxLayout;

    addSlider(tr("test"),tr("force"),-10.0,10.0);

    setLayout(layout_);

    //minimumSpinBox->setValue(0);
    //maximumSpinBox->setValue(20);
    //valueSpinBox->setValue(5);

    setWindowTitle(tr("Sliders"));
}

void Window::addSlider(const QString& group_name, const QString& data_name,
                       const double& min, const double& max)
{
    if(!sliders_.count(data_name))
    {
      sliders_[data_name] = new SlidersGroup(data_name, min, max);
      stacked_widget_->addWidget(sliders_[data_name]);

      //groups_[group_name] = createControls(data_name);

      //layout_->addWidget(groups_[group_name]);
      layout_->addWidget(stacked_widget_);
    }
    else {
      QMessageBox::warning(0, tr("An error occurred!"), tr("data_name already exists!"));
    }
}

QGroupBox* Window::createControls(const QString& title)
{
    QGroupBox *controlsGroup = new QGroupBox(title);

    minimumLabel = new QLabel(tr("Minimum value:"));
    maximumLabel = new QLabel(tr("Maximum value:"));
    valueLabel   = new QLabel(tr("Current value:"));

//    invertedAppearance = new QCheckBox(tr("Inverted appearance"));
//    invertedKeyBindings = new QCheckBox(tr("Inverted key bindings"));
//
////! [4] //! [5]
//    minimumSpinBox = new QSpinBox;
////! [5] //! [6]
//    minimumSpinBox->setRange(-100, 100);
//    minimumSpinBox->setSingleStep(1);
//
//    maximumSpinBox = new QSpinBox;
//    maximumSpinBox->setRange(-100, 100);
//    maximumSpinBox->setSingleStep(1);
//
//    valueSpinBox = new QSpinBox;
//    valueSpinBox->setRange(-100, 100);
//    valueSpinBox->setSingleStep(1);
//
//    orientationCombo = new QComboBox;
//    orientationCombo->addItem(tr("Horizontal slider-like widgets"));
//    orientationCombo->addItem(tr("Vertical slider-like widgets"));


//    connect(orientationCombo, SIGNAL(activated(int)),
//            stacked_widget_, SLOT(setCurrentIndex(int)));
//    connect(minimumSpinBox, SIGNAL(valueChanged(int)),
//            horizontalSliders, SLOT(setMinimum(int)));
//    connect(minimumSpinBox, SIGNAL(valueChanged(int)),
//            verticalSliders, SLOT(setMinimum(int)));
//    connect(maximumSpinBox, SIGNAL(valueChanged(int)),
//            horizontalSliders, SLOT(setMaximum(int)));
//    connect(maximumSpinBox, SIGNAL(valueChanged(int)),
//            verticalSliders, SLOT(setMaximum(int)));
//    connect(invertedAppearance, SIGNAL(toggled(bool)),
//            horizontalSliders, SLOT(invertAppearance(bool)));
//    connect(invertedAppearance, SIGNAL(toggled(bool)),
//            verticalSliders, SLOT(invertAppearance(bool)));
//    connect(invertedKeyBindings, SIGNAL(toggled(bool)),
//            horizontalSliders, SLOT(invertKeyBindings(bool)));
//    connect(invertedKeyBindings, SIGNAL(toggled(bool)),
//            verticalSliders, SLOT(invertKeyBindings(bool)));

    QGridLayout *controlsLayout = new QGridLayout;
    controlsLayout->addWidget(minimumLabel, 0, 0);
    controlsLayout->addWidget(maximumLabel, 1, 0);
    controlsLayout->addWidget(valueLabel, 2, 0);
 //   controlsLayout->addWidget(minimumSpinBox, 0, 1);
 //   controlsLayout->addWidget(maximumSpinBox, 1, 1);
 //   controlsLayout->addWidget(valueSpinBox, 2, 1);
 //   controlsLayout->addWidget(invertedAppearance, 0, 2);
 //   controlsLayout->addWidget(invertedKeyBindings, 1, 2);
 //   controlsLayout->addWidget(orientationCombo, 3, 0, 1, 3);
    controlsGroup->setLayout(controlsLayout);

    return controlsGroup;
}
//! [8]
