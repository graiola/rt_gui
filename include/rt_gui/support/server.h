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

    WindowServerHandler(Window* window, ros::NodeHandle& node, std::string srv_requested, std::string srv_provided, std::string client_name)
    {
        update_ = node.serviceClient<srv_t>("/"+client_name+"/"+srv_requested);
        add_    = node.advertiseService(client_name+"/"+srv_provided, &WindowServerHandler::addWidget, this);
        window_ = window;
    }

    virtual ~WindowServerHandler() {}

    virtual bool addWidget(typename srv_t::Request& req, typename srv_t::Response& res) = 0;

    bool update(srv_t& srv)
    {
        if(update_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            update_.call(srv);
            if(srv.response.resp == false)
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

    bool update(const std::string& group_name, const std::string& data_name, data_t value)
    {
        srv_t srv;
        srv.request.data_name  = data_name;
        srv.request.group_name = group_name;
        srv.request.value = value;
        return update(srv);
    }

protected:
    ros::ServiceServer add_;
    ros::ServiceClient update_;
    Window* window_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerServerHandler : public QObject, WindowServerHandler<rt_gui::Void,bool>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<TriggerServerHandler> Ptr;

    TriggerServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided, std::string client_name);

    bool addWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res);

public slots:
    bool updateButton(QString group_name, QString data_name);

signals:
    void addButton(const QString& group_name, const QString& data_name);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntServerHandler : public QObject, WindowServerHandler<rt_gui::Int,int>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<IntServerHandler> Ptr;

    IntServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided, std::string client_name);

    bool addWidget(rt_gui::Int::Request& req, rt_gui::Int::Response& res);

public slots:
    bool updateIntSlider(QString group_name, QString data_name, int value);

signals:
    void addIntSlider(const QString& group_name, const QString& data_name, const int& min, const int& max, const int& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleServerHandler : public QObject, WindowServerHandler<rt_gui::Double,double>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<DoubleServerHandler> Ptr;

    DoubleServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided, std::string client_name);

    bool addWidget(rt_gui::Double::Request& req, rt_gui::Double::Response& res);

public slots:
    bool updateDoubleSlider(QString group_name, QString data_name, double value);

signals:
    void addDoubleSlider(const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolServerHandler : public QObject, WindowServerHandler<rt_gui::Bool,bool>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<BoolServerHandler> Ptr;

    BoolServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided, std::string client_name);

    bool addWidget(rt_gui::Bool::Request& req, rt_gui::Bool::Response& res);

public slots:
    bool updateRadioButton(QString group_name, QString data_name, bool value);

signals:
    void addRadioButton(const QString& group_name, const QString& data_name, const bool& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListServerHandler : public QObject, WindowServerHandler<rt_gui::List,std::string>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<ListServerHandler> Ptr;

    ListServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided, std::string client_name);

    bool addWidget(rt_gui::List::Request& req, rt_gui::List::Response& res);

public slots:
    bool updateComboBox(QString group_name, QString data_name, QString value);

signals:
    void addComboBox(const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);
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
  {}

  DoubleServerHandler::Ptr double_h_;
  IntServerHandler::Ptr int_h_;
  BoolServerHandler::Ptr bool_h_;
  ListServerHandler::Ptr list_h_;
  TriggerServerHandler::Ptr trigger_h_;

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

  void init(const std::string server_name = RT_GUI_SERVER_NAME, QWidget* parent = nullptr)
  {
    ros_node_.reset(new RosNode(server_name,_ros_services.n_threads));

    window_      = new Window(QString::fromStdString(server_name),parent);
    add_client_  = ros_node_->getNode().advertiseService(_ros_services.add_client,     &RosServerNode::addClient,    this);
    remove_      = ros_node_->getNode().advertiseService(_ros_services.remove_service, &RosServerNode::removeWidget, this);

    QObject::connect(this,       SIGNAL(addClient(const QString &)),
                     this,       SLOT(createHandlers(const QString &)));

    QObject::connect(this,       SIGNAL(removeWidget(const QString &, const QString &)),
                     window_,    SLOT(removeWidget(const QString &, const QString &)));
  }

  bool removeWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
  {
    emit removeWidget(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
    res.resp = true;
    return res.resp;
  }

  bool addClient(rt_gui::Client::Request& req, rt_gui::Client::Response& res)
  {
    res.resp = false;
    if(!req.name.empty() && window_) // Client name
    {
      emit addClient(QString::fromStdString(req.name));
      res.resp = true;
      return res.resp;
    }
    return res.resp;
  }

public slots:
  void createHandlers(const QString &client_name)
  {
    std::string client_name_std                = client_name.toStdString();
    handlers_[client_name_std]                 = Handlers();
    handlers_[client_name_std].double_h_       = std::make_shared<DoubleServerHandler>(window_,ros_node_->getNode(),_ros_services.double_srvs.update,_ros_services.double_srvs.add,client_name_std);
    handlers_[client_name_std].int_h_          = std::make_shared<IntServerHandler>(window_,ros_node_->getNode(),_ros_services.int_srvs.update,_ros_services.int_srvs.add,client_name_std);
    handlers_[client_name_std].bool_h_         = std::make_shared<BoolServerHandler>(window_,ros_node_->getNode(),_ros_services.bool_srvs.update,_ros_services.bool_srvs.add,client_name_std);
    handlers_[client_name_std].list_h_         = std::make_shared<ListServerHandler>(window_,ros_node_->getNode(),_ros_services.list_srvs.update,_ros_services.list_srvs.add,client_name_std);
    handlers_[client_name_std].trigger_h_      = std::make_shared<TriggerServerHandler>(window_,ros_node_->getNode(),_ros_services.trigger_srvs.update,_ros_services.trigger_srvs.add,client_name_std);
  }

signals:
  void removeWidget(const QString &group_name, const QString &data_name);
  void addClient(const QString &client_name);

private:

  RosServerNode(const RosServerNode&)= delete;
  RosServerNode& operator=(const RosServerNode&)= delete;

  Window* window_;
  std::unique_ptr<RosNode> ros_node_;
  ros::ServiceServer remove_;
  ros::ServiceServer add_client_;
  std::map<std::string,Handlers> handlers_;
};


} // namespace


#endif
