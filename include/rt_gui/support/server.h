#ifndef RT_GUI_SUPPORT_SERVER_H
#define RT_GUI_SUPPORT_SERVER_H

#include <rt_gui/support/common.h>
#include <rt_gui/qt_utils/window.h>

namespace rt_gui
{

template<typename srv_t, typename data_t>
class WindowServerHandler
{

public:

  typedef std::shared_ptr<WindowServerHandler> Ptr;

  WindowServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv)
  {
    //update_ = node.serviceClient<srv_t>("/"+client_name+"/"+srv_requested);
    //assert(!srv_provided_.empty());
    //assert(!srv_requested_.empty());
    add_srv_      = add_srv;
    update_srv_   = update_srv;
    feedback_srv_ = feedback_srv;
    node_         = node;
    add_          = node_.advertiseService(add_srv, &WindowServerHandler::addWidget, this);
    feedback_     = node_.advertiseService(feedback_srv, &WindowServerHandler::feedback, this);
    window_       = window;
  }

  virtual ~WindowServerHandler() {}

  virtual bool addWidget(typename srv_t::Request& req, typename srv_t::Response& res) = 0;

  virtual bool feedback(typename srv_t::Request& /*req*/, typename srv_t::Response& /*res*/) {}

  bool update(srv_t& srv)
  {
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

  bool update(const std::string& client_name, const std::string& group_name, const std::string& data_name, data_t value)
  {
    srv_t srv;
    srv.request.client_name = client_name;
    srv.request.data_name   = data_name;
    srv.request.group_name  = group_name;
    srv.request.value       = value;
    return update(srv);
  }

  bool update(const std::string& client_name, const std::string& group_name, const std::string& data_name, data_t value, data_t& actual_value)
  {
    srv_t srv;
    srv.request.client_name = client_name;
    srv.request.data_name   = data_name;
    srv.request.group_name  = group_name;
    srv.request.value       = value;
    bool res                = update(srv);
    actual_value            = srv.response.resp;
    return res;
  }

protected:

  ros::NodeHandle node_;
  std::string add_srv_;
  std::string update_srv_;
  std::string feedback_srv_;
  ros::ServiceServer add_;
  ros::ServiceServer feedback_;
  Window* window_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerServerHandler : public QObject, WindowServerHandler<rt_gui::Void,bool>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<TriggerServerHandler> Ptr;

  TriggerServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res);

private slots:
  bool updateButton(QString client_name, QString group_name, QString data_name);

signals:
  void addButton(const QString& client_name, const QString& group_name, const QString& data_name);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntServerHandler : public QObject, WindowServerHandler<rt_gui::Int,int>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<IntServerHandler> Ptr;

  IntServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Int::Request& req, rt_gui::Int::Response& res);

public slots:
  bool updateIntSlider(QString client_name, QString group_name, QString data_name, int value);

signals:
  void addIntSlider(const QString& client_name, const QString& group_name, const QString& data_name, const int& min, const int& max, const int& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleServerHandler : public QObject, WindowServerHandler<rt_gui::Double,double>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<DoubleServerHandler> Ptr;

  DoubleServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Double::Request& req, rt_gui::Double::Response& res);

public slots:
  bool updateDoubleSlider(QString client_name, QString group_name, QString data_name, double value);

signals:
  void addDoubleSlider(const QString& client_name, const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolServerHandler : public QObject, WindowServerHandler<rt_gui::Bool,bool>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<BoolServerHandler> Ptr;

  BoolServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Bool::Request& req, rt_gui::Bool::Response& res);

public slots:
  bool updateRadioButton(QString client_name, QString group_name, QString data_name, bool value);

signals:
  void addRadioButton(const QString& client_name, const QString& group_name, const QString& data_name, const bool& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListServerHandler : public QObject, WindowServerHandler<rt_gui::List,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<ListServerHandler> Ptr;

  ListServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::List::Request& req, rt_gui::List::Response& res);

public slots:
  bool updateComboBox(QString client_name, QString group_name, QString data_name, QString value);

signals:
  void addComboBox(const QString& client_name, const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TextServerHandler : public QObject, WindowServerHandler<rt_gui::Text,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<TextServerHandler> Ptr;

  TextServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Text::Request& req, rt_gui::Text::Response& res);

public slots:
  bool updateText(QString client_name, QString group_name, QString data_name, QString value);

signals:
  void addText(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LabelServerHandler : public QObject, WindowServerHandler<rt_gui::Text,std::string>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<LabelServerHandler> Ptr;

  LabelServerHandler(Window* window, ros::NodeHandle& node, std::string add_srv, std::string update_srv, std::string feedback_srv);

  bool addWidget(rt_gui::Text::Request& req, rt_gui::Text::Response& res);

  bool feedback(rt_gui::Text::Request& req, rt_gui::Text::Response& res);

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

  void init(ros::NodeHandle& nh, const std::string server_name = RT_GUI_SERVER_NAME, QWidget* parent = nullptr)
  {

    window_      = new Window(QString::fromStdString(server_name),parent);
    remove_      = nh.advertiseService(_ros_services.remove_service, &RosServerNode::removeWidget, this);

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
    init(ros_node_->getNode(),server_name,parent);
  }

  bool removeWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
  {
    emit removeWidget(QString::fromStdString(req.client_name),QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
    res.resp = true;
    return res.resp;
  }

signals:
  void removeWidget(const QString &client_name, const QString &group_name, const QString &data_name);

private:

  RosServerNode(const RosServerNode&)= delete;
  RosServerNode& operator=(const RosServerNode&)= delete;

  Window* window_;
  std::unique_ptr<RosNode> ros_node_;
  ros::ServiceServer remove_;
  ros::ServiceServer add_client_;
  Handlers handlers_;
};


} // namespace


#endif
