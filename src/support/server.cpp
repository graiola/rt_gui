#include "rt_gui/support/server.h"

using namespace rt_gui;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TriggerServerHandler::TriggerServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
  :WindowServerHandler<rt_gui::Void,bool>(window,node,srv_requested,srv_provided)
{

  QObject::connect(this,    SIGNAL(addButton(const QString&, const QString&)),
                   window_, SLOT(addButton(const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateButton(QString, QString)),
                   this,    SLOT(updateButton(QString, QString)));
}

bool TriggerServerHandler::addWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
{
  emit addButton(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
  // FIXME add a proper error handling
  res.resp = true;
  return res.resp;
}

bool TriggerServerHandler::updateButton(QString group_name, QString data_name)
{
  rt_gui::Void srv;
  srv.request.data_name  = data_name.toStdString();
  srv.request.group_name = group_name.toStdString();
  if(update_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
  {
    update_.call(srv);
    if(srv.response.resp == false)
      throw std::runtime_error("RtGuiClient::update::resp is false!");
  }
  else
  {
    throw std::runtime_error("RtGuiClient::update service is not available!");
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IntServerHandler::IntServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
  :WindowServerHandler<rt_gui::Int,int>(window,node,srv_requested,srv_provided)
{
  QObject::connect(this,    SIGNAL(addIntSlider(const QString&, const QString&, const int&, const int&, const int&)),
                   window_, SLOT(addIntSlider(const QString&, const QString&, const int&, const int&, const int&)));

  QObject::connect(window_, SIGNAL(updateIntSlider(QString, QString, int)),
                   this,    SLOT(updateIntSlider(QString, QString, int)));
}

bool IntServerHandler::addWidget(rt_gui::Int::Request& req, rt_gui::Int::Response& res)
{
  emit addIntSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
  // FIXME add a proper error handling
  res.resp = true;
  return res.resp;
}


bool IntServerHandler::updateIntSlider(QString group_name, QString data_name, int value)
{
  return update(group_name.toStdString(),data_name.toStdString(),value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DoubleServerHandler::DoubleServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
  :WindowServerHandler<rt_gui::Double,double>(window,node,srv_requested,srv_provided)
{
  QObject::connect(this,    SIGNAL(addDoubleSlider(const QString&, const QString&, const double&, const double&, const double&)),
                   window_, SLOT(addDoubleSlider(const QString&, const QString&, const double&, const double&, const double&)));

  QObject::connect(window_, SIGNAL(updateDoubleSlider(QString, QString, double)),
                   this,    SLOT(updateDoubleSlider(QString, QString, double)));
}

bool DoubleServerHandler::addWidget(rt_gui::Double::Request& req, rt_gui::Double::Response& res)
{
  emit addDoubleSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
  // FIXME add a proper error handling
  res.resp = true;
  return res.resp;
}

bool DoubleServerHandler::updateDoubleSlider(QString group_name, QString data_name, double value)
{
  return update(group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoolServerHandler::BoolServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
  :WindowServerHandler<rt_gui::Bool,bool>(window,node,srv_requested,srv_provided)
{
  QObject::connect(this,    SIGNAL(addRadioButton(const QString&, const QString&, const bool&)),
                   window_, SLOT(addRadioButton(const QString&, const QString&, const bool&)));

  QObject::connect(window_, SIGNAL(updateRadioButton(QString, QString, bool)),
                   this,    SLOT(updateRadioButton(QString, QString, bool)));
}

bool BoolServerHandler::addWidget(rt_gui::Bool::Request& req, rt_gui::Bool::Response& res)
{
  emit addRadioButton(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.value);
  // FIXME add a proper error handling
  res.resp = true;
  return res.resp;
}

bool BoolServerHandler::updateRadioButton(QString group_name, QString data_name, bool value)
{
  return update(group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ListServerHandler::ListServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
  :WindowServerHandler<rt_gui::List,std::string>(window,node,srv_requested,srv_provided)
{
  QObject::connect(this,    SIGNAL(addComboBox(const QString&, const QString&, const QStringList&, const QString&)),
                   window_, SLOT(addComboBox(const QString&, const QString&, const QStringList&, const QString&)));

  QObject::connect(window_, SIGNAL(updateComboBox(QString, QString, QString)),
                   this,    SLOT(updateComboBox(QString, QString, QString)));
}

bool ListServerHandler::addWidget(rt_gui::List::Request& req, rt_gui::List::Response& res)
{
  QStringList list;
  for(unsigned int i=0;i<req.list.size();i++)
    list.push_back(QString::fromStdString(req.list[i]));
  emit addComboBox(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),list,QString::fromStdString(req.value));
  // FIXME add a proper error handling
  res.resp = true;
  return res.resp;
}

bool ListServerHandler::updateComboBox(QString group_name, QString data_name, QString value)
{
  return update(group_name.toStdString(),data_name.toStdString(),value.toStdString());
}


