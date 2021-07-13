#include "slidersgroup.h"
#include "window.h"


Window::Window()
{
  main_layout_ = new QVBoxLayout;
  tabs_ = new QTabWidget;

  setLayout(main_layout_);

  setWindowTitle(tr("rt_gui"));
}

void Window::addSlider(const QString& group_name, const QString& data_name,
                       const double& min, const double& max)
{
  if(sliders_.count(group_name) == 0)
    sliders_[group_name] = new SlidersGroup(group_name);

  Slider* slider = new Slider(data_name,min,max);
  sliders_[group_name]->addSlider(slider);
}

void Window::createTabs()
{
  const auto& map = sliders_.toStdMap();
  for(auto tmp : map)
  {
    tabs_->addTab(tmp.second,tmp.first);
  }
  main_layout_->addWidget(tabs_);
}
