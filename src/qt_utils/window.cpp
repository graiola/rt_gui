#include "qt_utils/slider.h"
#include "qt_utils/radio_button.h"
#include "qt_utils/window.h"

WidgetsGroup::WidgetsGroup(const QString &title,
                           QWidget *parent)
  : QGroupBox(title, parent)
{
  layout_ = new QBoxLayout(QBoxLayout::TopToBottom);
  setLayout(layout_);
}

void WidgetsGroup::add(QWidget *widget)
{
  layout_->addWidget(widget);
}

Window::Window(const QString& title)
{
  main_layout_ = new QVBoxLayout;
  tabs_ = new QTabWidget;

  setLayout(main_layout_);

  setWindowTitle(title);
}

bool Window::checkIfDuplicated(const widgets_group_map_t& map, const QString& group_name, const QString& data_name)
{
  if(widgets_group_.count(group_name) == 0)
    widgets_group_[group_name] = new WidgetsGroup(group_name);

  bool duplicated = false;
  for (int i = 0; i < map[group_name]->layout()->count(); ++i)
  {
    QWidget *widget = map[group_name]->layout()->itemAt(i)->widget();
    if(widget != Q_NULLPTR && widget->objectName() == data_name)
    {
      duplicated = true;
      break;
    }
  }
  if(duplicated)
     qDebug() << "Widget in: "<< group_name << ", " << data_name <<" already exists!";

  return duplicated;
}

void Window::addSlider(const QString& group_name, const QString& data_name,
                       const double& min, const double& max, const double& init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    Slider* slider = new Slider(group_name,data_name,min,max,init);
    widgets_group_[group_name]->add(slider);

    QObject::connect(slider, SIGNAL(valueChanged(double)),
                     this,   SLOT(sliderChanged(double)));

    createTabs();
  }
}

void Window::addRadioButton(const QString &group_name, const QString &data_name, const bool &init)
{

  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    RadioButton* radio_button = new RadioButton(group_name,data_name,init);
    widgets_group_[group_name]->add(radio_button);

    QObject::connect(radio_button, SIGNAL(valueChanged(bool)),
                     this,   SLOT(radioButtonChanged(bool)));

    createTabs();
  }
}

void Window::sliderChanged(double /*value*/)
{
  Slider* slider = qobject_cast<Slider*>(sender());
  if(slider!=Q_NULLPTR)
    emit updateSlider(slider->getGroupName(),slider->getDataName(),slider->getValue());
}

void Window::radioButtonChanged(bool /*value*/)
{
  RadioButton* radio = qobject_cast<RadioButton*>(sender());
  if(radio!=Q_NULLPTR)
    emit updateRadioButton(radio->getGroupName(),radio->getDataName(),radio->getValue());
}

void Window::createTabs()
{
  const auto& map = widgets_group_.toStdMap();
  for(auto tmp : map)
  {
    tabs_->addTab(tmp.second,tmp.first);
  }
  main_layout_->addWidget(tabs_);
  show();
}
