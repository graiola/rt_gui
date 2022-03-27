#ifndef SUPPORT_SERVER_H
#define SUPPORT_SERVER_H

#include "rt_gui/support/common.h"
#include "qt_utils/window.h"

namespace rt_gui
{

template<typename srv_t, typename data_t>
class WindowServerHandler
{

public:

    typedef std::shared_ptr<WindowServerHandler> Ptr;

    WindowServerHandler(Window* window, ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
    {
        update_ = node.serviceClient<srv_t>("/" RT_GUI_CLIENT_NAME "/"+srv_requested);
        add_ = node.advertiseService(srv_provided, &WindowServerHandler::addWidget, this);
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
                throw std::runtime_error("RtGuiClient::update::resp is false!");
        }
        else
        {
            throw std::runtime_error("RtGuiClient::update service is not available!");
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

    TriggerServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided);


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

    IntServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided);

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

    DoubleServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided);

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

    BoolServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided);

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

    ListServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided);

    bool addWidget(rt_gui::List::Request& req, rt_gui::List::Response& res);

public slots:
    bool updateComboBox(QString group_name, QString data_name, QString value);

signals:
    void addComboBox(const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);
};


} // namespace


#endif
