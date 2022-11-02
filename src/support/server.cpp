#include <rt_gui/support/server.h>

using namespace rt_gui;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TriggerServerHandler::TriggerServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Void,bool>(window,node,srv_provided,srv_requested)
{

  QObject::connect(this,    SIGNAL(addButton(const QString&, const QString&, const QString&)),
                   window_, SLOT(addButton(const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateButton(QString, QString, QString)),
                   this,    SLOT(updateButton(QString, QString, QString)));
}

bool TriggerServerHandler::addWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& /*res*/)
{
  emit addButton(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
  // FIXME add a proper error handling
  return true;
}

bool TriggerServerHandler::updateButton(QString client_name, QString group_name, QString data_name)
{
  rt_gui::Void srv;
  srv.request.client_name = client_name.toStdString();
  srv.request.data_name   = data_name.toStdString();
  srv.request.group_name  = group_name.toStdString();

  std::string service = "/"+srv.request.client_name+"/"+srv_requested_;
  if(ros::service::waitForService(service,ros::Duration(_ros_services.wait_service_secs)))
  {
    if(!ros::service::call(service,srv))
    {
      ROS_WARN("RtGuiClient::update::resp is false!");
      return false;
    }
  }
  else
  {
    ROS_WARN("RtGuiClient::update service is not available!");
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IntServerHandler::IntServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Int,int>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)),
                   window_, SLOT(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)));

  QObject::connect(window_, SIGNAL(updateIntSlider(QString, QString, QString, int)),
                   this,    SLOT(updateIntSlider(QString, QString, QString, int)));
}

bool IntServerHandler::addWidget(rt_gui::Int::Request& req, rt_gui::Int::Response& /*res*/)
{
  emit addIntSlider(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
  // FIXME add a proper error handling
  return true;
}


bool IntServerHandler::updateIntSlider(QString client_name, QString group_name, QString data_name, int value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DoubleServerHandler::DoubleServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Double,double>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)),
                   window_, SLOT(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)));

  QObject::connect(window_, SIGNAL(updateDoubleSlider(QString, QString, QString, double)),
                   this,    SLOT(updateDoubleSlider(QString, QString, QString, double)));
}

bool DoubleServerHandler::addWidget(rt_gui::Double::Request& req, rt_gui::Double::Response& /*res*/)
{
  emit addDoubleSlider(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
  // FIXME add a proper error handling
  return true;
}

bool DoubleServerHandler::updateDoubleSlider(QString client_name, QString group_name, QString data_name, double value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoolServerHandler::BoolServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Bool,bool>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addRadioButton(const QString&, const QString&, const QString&, const bool&)),
                   window_, SLOT(addRadioButton(const QString&, const QString&, const QString&, const bool&)));

  QObject::connect(window_, SIGNAL(updateRadioButton(QString, QString, QString, bool)),
                   this,    SLOT(updateRadioButton(QString, QString, QString, bool)));
}

bool BoolServerHandler::addWidget(rt_gui::Bool::Request& req, rt_gui::Bool::Response& /*res*/)
{
  emit addRadioButton(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.value);
  // FIXME add a proper error handling
  return true;
}

bool BoolServerHandler::updateRadioButton(QString client_name, QString group_name, QString data_name, bool value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ListServerHandler::ListServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::List,std::string>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)),
                   window_, SLOT(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)));

  QObject::connect(window_, SIGNAL(updateComboBox(QString, QString, QString, QString)),
                   this,    SLOT(updateComboBox(QString, QString, QString, QString)));
}

bool ListServerHandler::addWidget(rt_gui::List::Request& req, rt_gui::List::Response& /*res*/)
{
  QStringList list;
  for(unsigned int i=0;i<req.list.size();i++)
    list.push_back(QString::fromStdString(req.list[i]));
  emit addComboBox(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),list,QString::fromStdString(req.value));
  // FIXME add a proper error handling
  return true;
}

bool ListServerHandler::updateComboBox(QString client_name, QString group_name, QString data_name, QString value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value.toStdString());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TextServerHandler::TextServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Text,std::string>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addText(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addText(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateText(QString, QString, QString, QString)),
                   this,    SLOT(updateText(QString, QString, QString, QString)));
}

bool TextServerHandler::addWidget(rt_gui::Text::Request& req, rt_gui::Text::Response& /*res*/)
{
  emit addText(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),QString::fromStdString(req.value));
  // FIXME add a proper error handling
  return true;
}

bool TextServerHandler::updateText(QString client_name, QString group_name, QString data_name, QString value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value.toStdString());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LabelServerHandler::LabelServerHandler(Window* window, ros::NodeHandle& node, std::string srv_provided, std::string srv_requested)
  :WindowServerHandler<rt_gui::Text,std::string>(window,node,srv_provided,srv_requested)
{
  QObject::connect(this,    SIGNAL(addLabel(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addLabel(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateLabel(QString, QString, QString, QString, QString&)),
                   this,    SLOT(updateLabel(QString, QString, QString, QString, QString&)));
}

bool LabelServerHandler::addWidget(rt_gui::Text::Request& req, rt_gui::Text::Response& /*res*/)
{
  emit addLabel(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),QString::fromStdString(req.value));
  // FIXME add a proper error handling
  return true;
}

bool LabelServerHandler::updateLabel(QString client_name, QString group_name, QString data_name, QString value, QString &actual_value)
{
  std::string actual_string_std;
  bool res = update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value.toStdString(),actual_string_std);
  actual_value = QString::fromStdString(actual_string_std);
  return res;
}



