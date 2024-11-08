#ifndef RT_GUI_ROS_SUPPORT_SERVER_H
#define RT_GUI_ROS_SUPPORT_SERVER_H

#include <rt_gui_ros/support/ros_node.h>
#include <rt_gui_core/support/common.h>
#include <rt_gui_core/qt_utils/window.h>

#include <memory>
#include <functional>

namespace rt_gui
{

using namespace std::placeholders;

template<typename srv_t, typename data_t>
class WindowServerHandler
{

public:

  typedef std::shared_ptr<WindowServerHandler> Ptr;

  WindowServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  {
    add_srv_      = add_srv;
    update_srv_   = update_srv;
    feedback_srv_ = feedback_srv;
    node_         = node;
    add_          = node_->create_service<srv_t>("/" + std::string(node_->get_name()) + "/" + add_srv, std::bind(&WindowServerHandler::addWidget, this, std::placeholders::_1, std::placeholders::_2));
    feedback_     = node_->create_service<srv_t>("/" + std::string(node_->get_name()) + "/" + feedback_srv, std::bind(&WindowServerHandler::feedback, this, std::placeholders::_1, std::placeholders::_2));
    window_       = window;
  }

  virtual ~WindowServerHandler() {}

  virtual bool addWidget(typename srv_t::Request::Ptr req, typename srv_t::Response::Ptr res) = 0;

  virtual bool feedback(typename srv_t::Request::Ptr /*req*/, typename srv_t::Response::Ptr /*res*/) {}

  bool update(typename srv_t::Request srv)
  {

    std::string service = "/"+srv.client_name+"/"+update_srv_;

    typename rclcpp::Client<srv_t>::SharedPtr client = node_->create_client<srv_t>(service);

    //if(!client->wait_for_service(std::chrono::duration<double>(_ros_services.wait_service_secs)))
    //{
      auto result = client->async_send_request(std::make_shared<typename srv_t::Request>(srv));
      result.wait_for(std::chrono::duration<double>(_ros_services.wait_service_secs));
      // FIXME
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

  bool update(const std::string& client_name, const std::string& group_name, const std::string& data_name, data_t value)
  {
    typename srv_t::Request srv_req;
    srv_req.client_name = client_name;
    srv_req.data_name   = data_name;
    srv_req.group_name  = group_name;
    srv_req.value       = value;
    return update(srv_req);
  }

  bool update(const std::string& client_name, const std::string& group_name, const std::string& data_name, data_t value, data_t& actual_value)
  {
    typename srv_t::Request  srv_req;
    typename srv_t::Response srv_resp;
    srv_req.client_name = client_name;
    srv_req.data_name   = data_name;
    srv_req.group_name  = group_name;
    srv_req.value       = value;
    bool res            = update(srv_req);
    actual_value        = srv_resp.resp;
    return res;
  }

protected:

  std::shared_ptr<rclcpp::Node> node_;
  std::string add_srv_;
  std::string update_srv_;
  std::string feedback_srv_;
  typename rclcpp::Service<srv_t>::SharedPtr add_;
  typename rclcpp::Service<srv_t>::SharedPtr feedback_;
  Window* window_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Void,bool>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<TriggerServerHandler> Ptr;

  TriggerServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Void::Request::Ptr req, rt_gui_msgs::srv::Void::Response::Ptr res);

private slots:
  bool updateButton(QString client_name, QString group_name, QString data_name);

signals:
  void addButton(const QString& client_name, const QString& group_name, const QString& data_name);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Int,int>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<IntServerHandler> Ptr;

  IntServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Int::Request::Ptr req, rt_gui_msgs::srv::Int::Response::Ptr res);

public slots:
  bool updateIntSlider(QString client_name, QString group_name, QString data_name, int value);

signals:
  void addIntSlider(const QString& client_name, const QString& group_name, const QString& data_name, const int& min, const int& max, const int& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Double,double>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<DoubleServerHandler> Ptr;

  DoubleServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Double::Request::Ptr req, rt_gui_msgs::srv::Double::Response::Ptr res);

public slots:
  bool updateDoubleSlider(QString client_name, QString group_name, QString data_name, double value);

signals:
  void addDoubleSlider(const QString& client_name, const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Bool,bool>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<BoolServerHandler> Ptr;

  BoolServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Bool::Request::Ptr req, rt_gui_msgs::srv::Bool::Response::Ptr res);

public slots:
  bool updateRadioButton(QString client_name, QString group_name, QString data_name, bool value);

signals:
  void addRadioButton(const QString& client_name, const QString& group_name, const QString& data_name, const bool& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::List,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<ListServerHandler> Ptr;

  ListServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::List::Request::Ptr req, rt_gui_msgs::srv::List::Response::Ptr res);

public slots:
  bool updateComboBox(QString client_name, QString group_name, QString data_name, QString value);

signals:
  void addComboBox(const QString& client_name, const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TextServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Text,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<TextServerHandler> Ptr;

  TextServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr res);

public slots:
  bool updateText(QString client_name, QString group_name, QString data_name, QString value);

signals:
  void addText(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LabelServerHandler : public QObject, WindowServerHandler<rt_gui_msgs::srv::Text,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<LabelServerHandler> Ptr;

  LabelServerHandler(Window* window, std::shared_ptr<rclcpp::Node> node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr res);

  bool feedback(rt_gui_msgs::srv::Text::Request::Ptr req, rt_gui_msgs::srv::Text::Response::Ptr res);

public slots:
  bool updateLabel(QString client_name, QString group_name, QString data_name, QString value, QString& actual_value);

signals:
  void addLabel(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
  void labelFeedback(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct Handlers
{
public:

  Handlers()
    :double_h_(nullptr)
    ,int_h_(nullptr)
    ,bool_h_(nullptr)
    ,list_h_(nullptr)
    ,trigger_h_(nullptr)
    ,text_h_(nullptr)
    ,label_h_(nullptr)
  {}

  DoubleServerHandler::Ptr double_h_;
  IntServerHandler::Ptr int_h_;
  BoolServerHandler::Ptr bool_h_;
  ListServerHandler::Ptr list_h_;
  TriggerServerHandler::Ptr trigger_h_;
  TextServerHandler::Ptr text_h_;
  LabelServerHandler::Ptr label_h_;

};

/// \brief The RosServerNode class
class RosServerNode : public QObject
{
  Q_OBJECT

public:

  RosServerNode()
  {
  }

  ~RosServerNode()
  {
  }

  void init(std::shared_ptr<rclcpp::Node> nh, const std::string server_name = RT_GUI_SERVER_NAME, QWidget* parent = nullptr)
  {

    window_      = new Window(QString::fromStdString(server_name),parent);
    remove_      = nh->create_service<rt_gui_msgs::srv::Void>("/" + server_name + "/" + _ros_services.remove_service, std::bind(&RosServerNode::removeWidgetCb, this, std::placeholders::_1, std::placeholders::_2));

    handlers_                 = Handlers();
    handlers_.double_h_       = std::make_shared<DoubleServerHandler> ( window_,nh,  _ros_services.double_srvs.add ,  _ros_services.double_srvs.update  ,  _ros_services.double_srvs.feedback  );
    handlers_.int_h_          = std::make_shared<IntServerHandler>    ( window_,nh,  _ros_services.int_srvs.add    ,  _ros_services.int_srvs.update     ,  _ros_services.int_srvs.feedback     );
    handlers_.bool_h_         = std::make_shared<BoolServerHandler>   ( window_,nh,  _ros_services.bool_srvs.add   ,  _ros_services.bool_srvs.update    ,  _ros_services.bool_srvs.feedback    );
    handlers_.list_h_         = std::make_shared<ListServerHandler>   ( window_,nh,  _ros_services.list_srvs.add   ,  _ros_services.list_srvs.update    ,  _ros_services.list_srvs.feedback    );
    handlers_.trigger_h_      = std::make_shared<TriggerServerHandler>( window_,nh,  _ros_services.trigger_srvs.add,  _ros_services.trigger_srvs.update ,  _ros_services.trigger_srvs.feedback );
    handlers_.text_h_         = std::make_shared<TextServerHandler>   ( window_,nh,  _ros_services.text_srvs.add   ,  _ros_services.text_srvs.update    ,  _ros_services.text_srvs.feedback    );
    handlers_.label_h_        = std::make_shared<LabelServerHandler>  ( window_,nh,  _ros_services.label_srvs.add  ,  _ros_services.label_srvs.update   ,  _ros_services.label_srvs.feedback   );

    QObject::connect(this,       SIGNAL(removeWidget(const QString &, const QString &, const QString &)),
                     window_,    SLOT(removeWidget(const QString &, const QString &, const QString &)));
  }

  void init(const std::string server_name = RT_GUI_SERVER_NAME, QWidget* parent = nullptr)
  {
    ros_node_.reset(new RosNode(server_name,_ros_services.n_threads));
    init(ros_node_->getNodePtr(),server_name,parent);
  }

  bool removeWidgetCb(rt_gui_msgs::srv::Void::Request::Ptr req, rt_gui_msgs::srv::Void::Response::Ptr res)
  {
    emit removeWidget(QString::fromStdString(req->client_name),QString::fromStdString(req->group_name),QString::fromStdString(req->data_name));
    res->resp = true;
    return res->resp;
  }

signals:
  void removeWidget(const QString &client_name, const QString &group_name, const QString &data_name);

private:

  RosServerNode(const RosServerNode&)= delete;
  RosServerNode& operator=(const RosServerNode&)= delete;

  Window* window_;
  std::unique_ptr<RosNode> ros_node_;
  rclcpp::Service<rt_gui_msgs::srv::Void>::SharedPtr remove_;
  rclcpp::Service<rt_gui_msgs::srv::Void>::SharedPtr add_client_;
  Handlers handlers_;
};


} // namespace


#endif
