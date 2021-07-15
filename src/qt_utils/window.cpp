#include "qt_utils/slidersgroup.h"
#include "qt_utils/window.h"

Window::Window(const QString& title)
{
  main_layout_ = new QVBoxLayout;
  tabs_ = new QTabWidget;

  setLayout(main_layout_);

  setWindowTitle(title);
}

void Window::addSlider(const QString& group_name, const QString& data_name,
                       const double& min, const double& max, const double& init)
{

  // FIXME add checks to ensure that there is only one 'data_name' per 'group_name'
  if(sliders_.count(group_name) == 0)
    sliders_[group_name] = new SlidersGroup(group_name);

  Slider* slider = new Slider(group_name,data_name,min,max,init);
  sliders_[group_name]->addSlider(slider);

  QObject::connect(slider, SIGNAL(valueChanged(double)),
                   this,   SLOT(valueChanged(double)));

  createTabs();
}

void Window::valueChanged(double /*value*/)
{
  Slider* slider = qobject_cast<Slider*>(sender());
  if(slider!=Q_NULLPTR)
    emit updateServer(slider->getGroupName(),slider->getDataName(),slider->getValue());
}

void Window::createTabs()
{
  const auto& map = sliders_.toStdMap();
  for(auto tmp : map)
  {
    tabs_->addTab(tmp.second,tmp.first);
  }
  main_layout_->addWidget(tabs_);
  show();
}
