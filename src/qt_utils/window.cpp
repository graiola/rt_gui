#include "qt_utils/slider.h"
#include "qt_utils/radio_button.h"
#include "qt_utils/window.h"
#include "qt_utils/combo_box.h"
#include "qt_utils/button.h"

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

void WidgetsGroup::remove(QWidget *widget)
{
  layout_->removeWidget(widget);
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

void Window::addButton(const QString& group_name, const QString& data_name)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    Button* button = new Button(group_name,data_name);
    widgets_group_[group_name]->add(button);
    QObject::connect(button, SIGNAL(valueChanged()),
                     this,   SLOT(buttonChanged()));
    createTabs();
  }
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

void Window::addComboBox(const QString& group_name, const QString& data_name, const QStringList& list, const QString& init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    ComboBox* combo_box = new ComboBox(group_name,data_name,list,init);
    widgets_group_[group_name]->add(combo_box);
    QObject::connect(combo_box, SIGNAL(valueChanged(QString)),
                     this,   SLOT(comboBoxChanged(QString)));
    createTabs();
  }
}

void Window::buttonChanged()
{
  Button* button = qobject_cast<Button*>(sender());
  if(button!=Q_NULLPTR)
    emit updateButton(button->getGroupName(),button->getDataName());
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

void Window::comboBoxChanged(QString /*value*/)
{
  ComboBox* combo = qobject_cast<ComboBox*>(sender());
  if(combo!=Q_NULLPTR)
    emit updateComboBox(combo->getGroupName(),combo->getDataName(),combo->getValue());
}

void Window::removeWidget(const QString &group_name, const QString &data_name)
{
  if(widgets_group_.count(group_name) != 0)
  {
    for (int i = 0; i < widgets_group_[group_name]->layout()->count(); ++i)
    {
      QWidget *widget = widgets_group_[group_name]->layout()->itemAt(i)->widget();
      if(widget != Q_NULLPTR && widget->objectName() == data_name)
      {
        widgets_group_[group_name]->remove(widget);
        main_layout_->removeWidget(widget);
        delete widget;
      }
    }
  }
  if(widgets_group_[group_name]->layout()->count() == 0)
    delete widgets_group_[group_name];
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
