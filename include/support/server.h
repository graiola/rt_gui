#ifndef SUPPORT_SERVER_H
#define SUPPORT_SERVER_H

#include <support/common.h>
#include <qt_utils/window.h>

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

    TriggerServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :WindowServerHandler<rt_gui::Void,bool>(window,node,srv_requested,srv_provided)
    {

        QObject::connect(this,    SIGNAL(addButton(const QString&, const QString&)),
                         window_, SLOT(addButton(const QString&, const QString&)));

        QObject::connect(window_, SIGNAL(updateButton(QString, QString)),
                         this,    SLOT(updateButton(QString, QString)));
    }

    bool addWidget(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
    {
        emit addButton(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name));
        // FIXME add a proper error handling
        res.resp = true;
        return res.resp;
    }

public slots:
    bool updateButton(QString group_name, QString data_name)
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

signals:
    void addButton(const QString& group_name, const QString& data_name);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntServerHandler : public QObject, WindowServerHandler<rt_gui::Int,int>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<IntServerHandler> Ptr;

    IntServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :WindowServerHandler<rt_gui::Int,int>(window,node,srv_requested,srv_provided)
    {
        QObject::connect(this,    SIGNAL(addIntSlider(const QString&, const QString&, const int&, const int&, const int&)),
                         window_, SLOT(addIntSlider(const QString&, const QString&, const int&, const int&, const int&)));

        QObject::connect(window_, SIGNAL(updateIntSlider(QString, QString, int)),
                         this,    SLOT(updateIntSlider(QString, QString, int)));
    }

    bool addWidget(rt_gui::Int::Request& req, rt_gui::Int::Response& res)
    {
        emit addIntSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
        // FIXME add a proper error handling
        res.resp = true;
        return res.resp;
    }

public slots:
    bool updateIntSlider(QString group_name, QString data_name, int value)
    {
        return update(group_name.toStdString(),data_name.toStdString(),value);
    }

signals:
    void addIntSlider(const QString& group_name, const QString& data_name, const int& min, const int& max, const int& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleServerHandler : public QObject, WindowServerHandler<rt_gui::Double,double>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<DoubleServerHandler> Ptr;

    DoubleServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :WindowServerHandler<rt_gui::Double,double>(window,node,srv_requested,srv_provided)
    {
        QObject::connect(this,    SIGNAL(addDoubleSlider(const QString&, const QString&, const double&, const double&, const double&)),
                         window_, SLOT(addDoubleSlider(const QString&, const QString&, const double&, const double&, const double&)));

        QObject::connect(window_, SIGNAL(updateDoubleSlider(QString, QString, double)),
                         this,    SLOT(updateDoubleSlider(QString, QString, double)));
    }

    bool addWidget(rt_gui::Double::Request& req, rt_gui::Double::Response& res)
    {
        emit addDoubleSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.value);
        // FIXME add a proper error handling
        res.resp = true;
        return res.resp;
    }

public slots:
    bool updateDoubleSlider(QString group_name, QString data_name, double value)
    {
        return update(group_name.toStdString(),data_name.toStdString(),value);
    }

signals:
    void addDoubleSlider(const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolServerHandler : public QObject, WindowServerHandler<rt_gui::Bool,bool>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<BoolServerHandler> Ptr;

    BoolServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :WindowServerHandler<rt_gui::Bool,bool>(window,node,srv_requested,srv_provided)
    {
        QObject::connect(this,    SIGNAL(addRadioButton(const QString&, const QString&, const bool&)),
                         window_, SLOT(addRadioButton(const QString&, const QString&, const bool&)));

        QObject::connect(window_, SIGNAL(updateRadioButton(QString, QString, bool)),
                         this,    SLOT(updateRadioButton(QString, QString, bool)));
    }

    bool addWidget(rt_gui::Bool::Request& req, rt_gui::Bool::Response& res)
    {
        emit addRadioButton(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.value);
        // FIXME add a proper error handling
        res.resp = true;
        return res.resp;
    }

public slots:
    bool updateRadioButton(QString group_name, QString data_name, bool value)
    {
        return update(group_name.toStdString(),data_name.toStdString(),value);
    }

signals:
    void addRadioButton(const QString& group_name, const QString& data_name, const bool& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListServerHandler : public QObject, WindowServerHandler<rt_gui::List,std::string>
{

    Q_OBJECT

public:

    typedef std::shared_ptr<ListServerHandler> Ptr;

    ListServerHandler(Window* window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :WindowServerHandler<rt_gui::List,std::string>(window,node,srv_requested,srv_provided)
    {
        QObject::connect(this,    SIGNAL(addComboBox(const QString&, const QString&, const QStringList&, const QString&)),
                         window_, SLOT(addComboBox(const QString&, const QString&, const QStringList&, const QString&)));

        QObject::connect(window_, SIGNAL(updateComboBox(QString, QString, QString)),
                         this,    SLOT(updateComboBox(QString, QString, QString)));
    }

    bool addWidget(rt_gui::List::Request& req, rt_gui::List::Response& res)
    {
        QStringList list;
        for(unsigned int i=0;i<req.list.size();i++)
            list.push_back(QString::fromStdString(req.list[i]));
        emit addComboBox(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),list,QString::fromStdString(req.value));
        // FIXME add a proper error handling
        res.resp = true;
        return res.resp;
    }

public slots:
    bool updateComboBox(QString group_name, QString data_name, QString value)
    {
        return update(group_name.toStdString(),data_name.toStdString(),value.toStdString());
    }

signals:
    void addComboBox(const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);
};


} // namespace


#endif
