#include <rt_gui_ros2/support/server.h>

using namespace rt_gui;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TriggerServerHandler::TriggerServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Void,bool>(window,node,add_srv,update_srv,feedback_srv)
{

  QObject::connect(this,    SIGNAL(addButton(const QString&, const QString&, const QString&)),
                   window_, SLOT(addButton(const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateButton(QString, QString, QString)),
                   this,    SLOT(updateButton(QString, QString, QString)));
}

bool TriggerServerHandler::addWidget(rt_gui_msgs::Void::Request& req, rt_gui_msgs::Void::Response& /*res*/)
{
  emit addButton(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
  // FIXME add a proper error handling
  return true;
}

bool TriggerServerHandler::updateButton(QString client_name, QString group_name, QString data_name)
{
  rt_gui_msgs::Void srv;
  srv.request.client_name = client_name.toStdString();
  srv.request.data_name   = data_name.toStdString();
  srv.request.group_name  = group_name.toStdString();

  std::string service = "/"+srv.request.client_name+"/"+update_srv_;
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

IntServerHandler::IntServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Int,int>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)),
                   window_, SLOT(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)));

  QObject::connect(window_, SIGNAL(updateIntSlider(QString, QString, QString, int)),
                   this,    SLOT(updateIntSlider(QString, QString, QString, int)));
}

bool IntServerHandler::addWidget(rt_gui_msgs::Int::Request& req, rt_gui_msgs::Int::Response& /*res*/)
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

DoubleServerHandler::DoubleServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Double,double>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)),
                   window_, SLOT(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)));

  QObject::connect(window_, SIGNAL(updateDoubleSlider(QString, QString, QString, double)),
                   this,    SLOT(updateDoubleSlider(QString, QString, QString, double)));
}

bool DoubleServerHandler::addWidget(rt_gui_msgs::Double::Request& req, rt_gui_msgs::Double::Response& /*res*/)
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

BoolServerHandler::BoolServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Bool,bool>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addRadioButton(const QString&, const QString&, const QString&, const bool&)),
                   window_, SLOT(addRadioButton(const QString&, const QString&, const QString&, const bool&)));

  QObject::connect(window_, SIGNAL(updateRadioButton(QString, QString, QString, bool)),
                   this,    SLOT(updateRadioButton(QString, QString, QString, bool)));
}

bool BoolServerHandler::addWidget(rt_gui_msgs::Bool::Request& req, rt_gui_msgs::Bool::Response& /*res*/)
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

ListServerHandler::ListServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::List,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)),
                   window_, SLOT(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)));

  QObject::connect(window_, SIGNAL(updateComboBox(QString, QString, QString, QString)),
                   this,    SLOT(updateComboBox(QString, QString, QString, QString)));
}

bool ListServerHandler::addWidget(rt_gui_msgs::List::Request& req, rt_gui_msgs::List::Response& /*res*/)
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

TextServerHandler::TextServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Text,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addText(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addText(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateText(QString, QString, QString, QString)),
                   this,    SLOT(updateText(QString, QString, QString, QString)));
}

bool TextServerHandler::addWidget(rt_gui_msgs::Text::Request& req, rt_gui_msgs::Text::Response& /*res*/)
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

LabelServerHandler::LabelServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::Text,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addLabel(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addLabel(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(this,    SIGNAL(labelFeedback(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(labelFeedback(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateLabel(QString, QString, QString, QString, QString&)),
                   this,    SLOT(updateLabel(QString, QString, QString, QString, QString&)));
}

bool LabelServerHandler::addWidget(rt_gui_msgs::Text::Request& req, rt_gui_msgs::Text::Response& /*res*/)
{
  emit addLabel(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),QString::fromStdString(req.value));
  // FIXME add a proper error handling
  return true;
}

bool LabelServerHandler::feedback(rt_gui_msgs::Text::Request& req, rt_gui_msgs::Text::Response& /*res*/)
{
  emit labelFeedback(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),QString::fromStdString(req.value));
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



