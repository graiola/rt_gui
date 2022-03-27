#ifndef SUPPORT_CLIENT_H
#define SUPPORT_CLIENT_H

#include "rt_gui/support/common.h"
#include <type_traits>

namespace rt_gui
{

template<class srv_t, class data_t>
class InterfaceHandler
{

public:

    typedef std::shared_ptr<InterfaceHandler> Ptr;

    typedef std::function<void(data_t)> fun_t;

    InterfaceHandler(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
    {
        server_    = node.advertiseService(srv_provided, &InterfaceHandler::update, this);
        client_    = node.serviceClient<srv_t>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
    }

    virtual ~InterfaceHandler() {}

    bool update(typename srv_t::Request& req, typename srv_t::Response& res)
    {
        res.resp = updateBuffer(req.group_name,req.data_name,req.value);
        return res.resp;
    }

    void add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t* data_ptr, bool sync)
    {
        srv_t srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        for(unsigned int i=0;i<list.size();i++)
            srv.request.list.push_back(list[i]);
        addRawData(group_name,data_name,data_ptr,srv,sync);
    }

    void add(const std::string& group_name, const std::string& data_name, const std::vector<data_t>& list, data_t data, fun_t fun, bool sync)
    {
        srv_t srv;
        srv.request.value = data;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        for(unsigned int i=0;i<list.size();i++)
            srv.request.list.push_back(list[i]);
        addCallback(group_name,data_name,data,fun,srv,sync);
    }

    void add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, bool sync)
    {
        srv_t srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        addRawData(group_name,data_name,data_ptr,srv,sync);
    }

    void add(const std::string& group_name, const std::string& data_name, data_t data, fun_t fun, bool sync)
    {
        srv_t srv;
        srv.request.value = data;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        addCallback(group_name,data_name,data,fun,srv,sync);
    }

    void add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t* data_ptr, bool sync)
    {
        srv_t srv;
        srv.request.min = min;
        srv.request.max = max;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        addRawData(group_name,data_name,data_ptr,srv,sync);
    }

    void add(const std::string& group_name, const std::string& data_name, const data_t& min, const data_t& max, data_t data, fun_t fun, bool sync)
    {
        srv_t srv;
        srv.request.min = min;
        srv.request.max = max;
        srv.request.value = data;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        addCallback(group_name,data_name,data,fun,srv,sync);
    }

    bool sync()
    {
        bool res = true;
        if(sync_mtx_.try_lock())
        {
            res = res && buffer_.sync();
            sync_mtx_.unlock();
        }
        return res;
    }

protected:

    void addRawData(const std::string& group_name, const std::string& data_name, data_t* data_ptr, srv_t& srv, bool sync)
    {
        if(this->client_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            this->client_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
            else
                buffer_.add(group_name,data_name,data_ptr,sync);
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

    void addCallback(const std::string& group_name, const std::string& data_name, data_t data, std::function<void(data_t)> fun, srv_t& srv, bool sync)
    {
        if(this->client_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            this->client_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
            else
                buffer_.add(group_name,data_name,data,fun,sync);
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

    bool updateBuffer(const std::string& group_name, const std::string& data_name, const decltype(srv_t::Request::value)& value)
    {
        sync_mtx_.lock();
        bool res = buffer_.update(group_name,data_name,value);
        sync_mtx_.unlock();
        return res;
    }

    ros::ServiceServer server_;
    ros::ServiceClient client_;
    CallbackBuffer<data_t> buffer_;
    std::mutex sync_mtx_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerHandler
{

public:

    typedef std::shared_ptr<TriggerHandler> Ptr;

    typedef std::function<void()> fun_t;

    TriggerHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    {
        server_    = node.advertiseService(srv_provided, &TriggerHandler::update, this);
        client_    = node.serviceClient<rt_gui::Void>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
    }

    void add(const std::string& group_name, const std::string& data_name, fun_t fun)
    {
        rt_gui::Void srv;
        assert(fun);
        fun_ = fun;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        if(this->client_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            this->client_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

    bool update(rt_gui::Void::Request& req, rt_gui::Void::Response& res)
    {
        fun_();
        res.resp = true;
        return res.resp;
    }

protected:

    fun_t fun_;
    ros::ServiceServer server_;
    ros::ServiceClient client_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntHandler : public InterfaceHandler<rt_gui::Int,int>
{

public:

    typedef std::shared_ptr<IntHandler> Ptr;

    IntHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<rt_gui::Int,int>(node,srv_requested,srv_provided) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleHandler : public InterfaceHandler<rt_gui::Double,double>
{

public:

    typedef std::shared_ptr<DoubleHandler> Ptr;

    DoubleHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<rt_gui::Double,double>(node,srv_requested,srv_provided) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolHandler : public InterfaceHandler<rt_gui::Bool,bool>
{

public:

    typedef std::shared_ptr<BoolHandler> Ptr;

    BoolHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<rt_gui::Bool,bool>(node,srv_requested,srv_provided) {}
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListHandler : public InterfaceHandler<rt_gui::List,std::string>
{

public:

    typedef std::shared_ptr<ListHandler> Ptr;

    ListHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<rt_gui::List,std::string>(node,srv_requested,srv_provided) {}
};

} // namespace


#endif
