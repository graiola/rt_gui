#include <rt_gui_ros/support/server.h>
#include <chrono>

using namespace rt_gui;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TriggerServerHandler::TriggerServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Void,bool>(window,node,add_srv,update_srv,feedback_srv)
{

  QObject::connect(this,    SIGNAL(addButton(const QString&, const QString&, const QString&)),
                   window_, SLOT(addButton(const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateButton(QString, QString, QString)),
                   this,    SLOT(updateButton(QString, QString, QString)));
}

bool TriggerServerHandler::addWidget(rt_gui_msgs::srv::Void::Request::Ptr req, rt_gui_msgs::srv::Void::Response::Ptr /*res*/)
{
  emit addButton(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name));
  // FIXME add a proper error handling
  return true;
}

bool TriggerServerHandler::updateButton(QString client_name, QString group_name, QString data_name)
{

  auto srv_req = std::make_shared<rt_gui_msgs::srv::Void::Request>();
  srv_req->client_name = client_name.toStdString();
  srv_req->data_name   = data_name.toStdString();
  srv_req->group_name  = group_name.toStdString();

  std::string service = "/"+srv_req->client_name+"/"+update_srv_;

  rclcpp::Client<rt_gui_msgs::srv::Void>::SharedPtr client =
      node_->create_client<rt_gui_msgs::srv::Void>(service);

  //if(!client->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
  //{
    auto result = client->async_send_request(srv_req);
    result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));
    //if(!result)
    //{
    //  RCLCPP_WARN("RtGuiClient::update::resp is false!");
    //  return false;
    //}
  //}
  //else
  //{
  //  RCLCPP_WARN(node_->get_logger(),"RtGuiClient::update service is not available!");
  //  return false;
  //}
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IntServerHandler::IntServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Int,int>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)),
                   window_, SLOT(addIntSlider(const QString&, const QString&, const QString&, const int&, const int&, const int&)));

  QObject::connect(window_, SIGNAL(updateIntSlider(QString, QString, QString, int)),
                   this,    SLOT(updateIntSlider(QString, QString, QString, int)));
}

bool IntServerHandler::addWidget(rt_gui_msgs::srv::Int::Request::Ptr req, rt_gui_msgs::srv::Int::Response::Ptr /*res*/)
{
  emit addIntSlider(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),req->min,req->max,req->value);
  // FIXME add a proper error handling
  return true;
}

bool IntServerHandler::updateIntSlider(QString client_name, QString group_name, QString data_name, int value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DoubleServerHandler::DoubleServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Double,double>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)),
                   window_, SLOT(addDoubleSlider(const QString&, const QString&, const QString&, const double&, const double&, const double&)));

  QObject::connect(window_, SIGNAL(updateDoubleSlider(QString, QString, QString, double)),
                   this,    SLOT(updateDoubleSlider(QString, QString, QString, double)));
}

bool DoubleServerHandler::addWidget(rt_gui_msgs::srv::Double::Request::Ptr req, rt_gui_msgs::srv::Double::Response::Ptr /*res*/)
{
  emit addDoubleSlider(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),req->min,req->max,req->value);
  // FIXME add a proper error handling
  return true;
}

bool DoubleServerHandler::updateDoubleSlider(QString client_name, QString group_name, QString data_name, double value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoolServerHandler::BoolServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Bool,bool>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addRadioButton(const QString&, const QString&, const QString&, const bool&)),
                   window_, SLOT(addRadioButton(const QString&, const QString&, const QString&, const bool&)));

  QObject::connect(window_, SIGNAL(updateRadioButton(QString, QString, QString, bool)),
                   this,    SLOT(updateRadioButton(QString, QString, QString, bool)));
}

bool BoolServerHandler::addWidget(rt_gui_msgs::srv::Bool::Request::Ptr req, rt_gui_msgs::srv::Bool::Response::Ptr /*res*/)
{
  emit addRadioButton(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),req->value);
  // FIXME add a proper error handling
  return true;
}

bool BoolServerHandler::updateRadioButton(QString client_name, QString group_name, QString data_name, bool value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ListServerHandler::ListServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::List,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)),
                   window_, SLOT(addComboBox(const QString&, const QString&, const QString&, const QStringList&, const QString&)));

  QObject::connect(window_, SIGNAL(updateComboBox(QString, QString, QString, QString)),
                   this,    SLOT(updateComboBox(QString, QString, QString, QString)));
}

bool ListServerHandler::addWidget(rt_gui_msgs::srv::List::Request::Ptr req, rt_gui_msgs::srv::List::Response::Ptr /*res*/)
{
  QStringList list;
  for(unsigned int i=0;i<req->list.size();i++)
    list.push_back(QString::fromStdString(req->list[i]));
  emit addComboBox(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),list,QString::fromStdString(req->value));
  // FIXME add a proper error handling
  return true;
}

bool ListServerHandler::updateComboBox(QString client_name, QString group_name, QString data_name, QString value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value.toStdString());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TextServerHandler::TextServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Text,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addText(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addText(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateText(QString, QString, QString, QString)),
                   this,    SLOT(updateText(QString, QString, QString, QString)));
}

bool TextServerHandler::addWidget(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr /*res*/)
{
  emit addText(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),QString::fromStdString(req->value));
  // FIXME add a proper error handling
  return true;
}

bool TextServerHandler::updateText(QString client_name, QString group_name, QString data_name, QString value)
{
  return update(client_name.toStdString(),group_name.toStdString(),data_name.toStdString(),value.toStdString());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LabelServerHandler::LabelServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  :WindowServerHandler<rt_gui_msgs::srv::Text,std::string>(window,node,add_srv,update_srv,feedback_srv)
{
  QObject::connect(this,    SIGNAL(addLabel(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(addLabel(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(this,    SIGNAL(labelFeedback(const QString&, const QString&, const QString&, const QString&)),
                   window_, SLOT(labelFeedback(const QString&, const QString&, const QString&, const QString&)));

  QObject::connect(window_, SIGNAL(updateLabel(QString, QString, QString, QString, QString&)),
                   this,    SLOT(updateLabel(QString, QString, QString, QString, QString&)));
}

bool LabelServerHandler::addWidget(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr /*res*/)
{
  emit addLabel(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),QString::fromStdString(req->value));
  // FIXME add a proper error handling
  return true;
}

bool LabelServerHandler::feedback(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr /*res*/)
{
  emit labelFeedback(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name),QString::fromStdString(req->value));
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



