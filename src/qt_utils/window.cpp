#include <rt_gui/qt_utils/double_slider.h>
#include <rt_gui/qt_utils/int_slider.h>
#include <rt_gui/qt_utils/radio_button.h>
#include <rt_gui/qt_utils/window.h>
#include <rt_gui/qt_utils/combo_box.h>
#include <rt_gui/qt_utils/button.h>
#include <rt_gui/qt_utils/text.h>
#include <rt_gui/qt_utils/label.h>

WidgetsGroup::WidgetsGroup(const QString& /*title*/,
                           QWidget *parent)
  : QScrollArea(parent)
{

  group_ = new QGroupBox(this);

  layout_ = new QBoxLayout(QBoxLayout::TopToBottom);
  layout_->setSpacing(0);
  layout_->setMargin(0);

  setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  setWidgetResizable(true);
  setWidget(group_);

  group_->setLayout(layout_);
}

QSize WidgetsGroup::sizeHint() const {
    QSize parentSize(QWidget::sizeHint());
    return QSize(parentSize.width() + 500, parentSize.height() + 500);
}

void WidgetsGroup::add(QWidget *widget)
{
  widget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  layout_->addWidget(widget);

  //adjustSize();
}

void WidgetsGroup::remove(QWidget *widget)
{
  layout_->removeWidget(widget);
}

QLayout *WidgetsGroup::getLayout()
{
  return group_->layout();
}

Window::Window(const QString& title, QWidget* parent)
  :QWidget (parent, Qt::Widget)
{
  main_layout_ = new QVBoxLayout(this);
  main_layout_->setSpacing(0);
  main_layout_->setMargin(0);
  tabs_ = new QTabWidget(this);
  tabs_->setMovable(true);

  if(parent!=nullptr)
    parent->setLayout(main_layout_);
  else
  {
    setWindowFlag(Qt::WindowType::Window);
    setLayout(main_layout_);
  }
  setWindowTitle(title);
}

bool Window::checkIfDuplicated(const widgets_group_map_t& map, const QString& group_name, const QString& data_name)
{
  if(widgets_group_.count(group_name) == 0)
    widgets_group_[group_name] = new WidgetsGroup(group_name);

  bool duplicated = false;

  for (int i = 0; i < map[group_name]->getLayout()->count(); ++i)
  {
    QWidget *widget = map[group_name]->getLayout()->itemAt(i)->widget();
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

void Window::addText(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    Text* text = new Text(client_name,group_name,data_name,placeholder);
    widgets_group_[group_name]->add(text);
    QObject::connect(text, SIGNAL(valueChanged(QString)),
                     this,   SLOT(textChanged(QString)));
    createTabs();
  }
}

void Window::addLabel(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    Label* label = new Label(client_name,group_name,data_name,placeholder);
    widgets_group_[group_name]->add(label);
    QObject::connect(label, SIGNAL(updateValue()),
                     this,   SLOT(labelChanged()));
    createTabs();
  }
}

void Window::addButton(const QString& client_name, const QString& group_name, const QString& data_name)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    Button* button = new Button(client_name,group_name,data_name);
    widgets_group_[group_name]->add(button);
    QObject::connect(button, SIGNAL(valueChanged()),
                     this,   SLOT(buttonChanged()));
    createTabs();
  }
}

void Window::addIntSlider(const QString& client_name, const QString& group_name, const QString& data_name,
                          const int& min, const int& max, const int& init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    IntSlider* int_slider = new IntSlider(client_name,group_name,data_name,min,max,init);
    widgets_group_[group_name]->add(int_slider);
    QObject::connect(int_slider, SIGNAL(valueChanged(int)),
                     this,   SLOT(intSliderChanged(int)));
    createTabs();
  }
}

void Window::addDoubleSlider(const QString& client_name, const QString& group_name, const QString& data_name,
                             const double& min, const double& max, const double& init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    DoubleSlider* double_slider = new DoubleSlider(client_name,group_name,data_name,min,max,init);
    widgets_group_[group_name]->add(double_slider);
    QObject::connect(double_slider, SIGNAL(valueChanged(double)),
                     this,   SLOT(doubleSliderChanged(double)));
    createTabs();
  }
}

void Window::addRadioButton(const QString& client_name, const QString &group_name, const QString &data_name, const bool &init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    RadioButton* radio_button = new RadioButton(client_name,group_name,data_name,init);
    widgets_group_[group_name]->add(radio_button);
    QObject::connect(radio_button, SIGNAL(valueChanged(bool)),
                     this,   SLOT(radioButtonChanged(bool)));
    createTabs();
  }
}

void Window::addComboBox(const QString& client_name, const QString& group_name, const QString& data_name, const QStringList& list, const QString& init)
{
  if(!checkIfDuplicated(widgets_group_,group_name,data_name))
  {
    ComboBox* combo_box = new ComboBox(client_name,group_name,data_name,list,init);
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
    emit updateButton(button->getClientName(),button->getGroupName(),button->getDataName());
}

void Window::textChanged(QString /*value*/)
{
  Text* text = qobject_cast<Text*>(sender());
  if(text!=Q_NULLPTR)
    emit updateText(text->getClientName(),text->getGroupName(),text->getDataName(),text->getValue());
}

void Window::labelChanged()
{
  Label* label = qobject_cast<Label*>(sender());
  QString actual_value;
  if(label!=Q_NULLPTR)
    emit updateLabel(label->getClientName(),label->getGroupName(),label->getDataName(),label->getValue(),actual_value);
  label->setValue(actual_value);
}

void Window::intSliderChanged(int /*value*/)
{
  IntSlider* int_slider = qobject_cast<IntSlider*>(sender());
  if(int_slider!=Q_NULLPTR)
    emit updateIntSlider(int_slider->getClientName(),int_slider->getGroupName(),int_slider->getDataName(),int_slider->getValue());
}

void Window::doubleSliderChanged(double /*value*/)
{
  DoubleSlider* double_slider = qobject_cast<DoubleSlider*>(sender());
  if(double_slider!=Q_NULLPTR)
    emit updateDoubleSlider(double_slider->getClientName(),double_slider->getGroupName(),double_slider->getDataName(),double_slider->getValue());
}

void Window::radioButtonChanged(bool /*value*/)
{
  RadioButton* radio = qobject_cast<RadioButton*>(sender());
  if(radio!=Q_NULLPTR)
    emit updateRadioButton(radio->getClientName(),radio->getGroupName(),radio->getDataName(),radio->getValue());
}

void Window::comboBoxChanged(QString /*value*/)
{
  ComboBox* combo = qobject_cast<ComboBox*>(sender());
  if(combo!=Q_NULLPTR)
    emit updateComboBox(combo->getClientName(),combo->getGroupName(),combo->getDataName(),combo->getValue());
}

void Window::removeWidget(const QString &/*client_name*/, const QString &group_name, const QString &data_name)
{
  if(widgets_group_.count(group_name) != 0)
  {
    for (int i = 0; i < widgets_group_[group_name]->getLayout()->count(); ++i)
    {
      QWidget *widget = widgets_group_[group_name]->getLayout()->itemAt(i)->widget();
      if(widget != Q_NULLPTR && widget->objectName() == data_name)
      {
        widgets_group_[group_name]->remove(widget);
        main_layout_->removeWidget(widget);
        delete widget;
      }
    }
  }
  if(widgets_group_[group_name]->getLayout()->count() == 0)
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
