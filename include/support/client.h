#ifndef SUPPORT_CLIENT_H
#define SUPPORT_CLIENT_H

#include <support/common.h>
#include <mutex>
#include <type_traits>

namespace rt_gui
{

template<class srv_t>
class InterfaceHandler
{

public:

    typedef std::shared_ptr<InterfaceHandler> Ptr;

    InterfaceHandler(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
    {
        server_    = node.advertiseService(srv_provided, &InterfaceHandler::update, this);
        client_    = node.serviceClient<srv_t>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
    }

    virtual ~InterfaceHandler() {}

    virtual bool update(typename srv_t::Request& req, typename srv_t::Response& res) = 0;

    virtual bool sync() = 0;

protected:

    ros::ServiceServer server_;
    ros::ServiceClient client_;
};

template <class data_t>
class Buffer {

public:

    Buffer() {}

    typedef std::pair<std::string,std::string> buffer_key_t;
    typedef std::pair<data_t*,data_t> buffer_value_t;
    typedef std::map<buffer_key_t, buffer_value_t> buffer_t;

    void add(const std::string& key1, const std::string& key2, data_t* data_ptr)
    {
        buffer_[buffer_key_t(key1,key2)] = buffer_value_t(data_ptr,*data_ptr);
    }

    void update(const std::string& key1, const std::string& key2, const data_t& value)
    {
        buffer_[buffer_key_t(key1,key2)].second = value;
    }

    void sync()
    {
        for(auto tmp_map : buffer_)
        {
            if(tmp_map.second.first!=nullptr) // The data pointer still exists
                *tmp_map.second.first = tmp_map.second.second;
        }
    }

private:
    buffer_t buffer_;

};

template<class srv_t, typename data_t>
class BufferHandler : public InterfaceHandler<srv_t>
{

public:

    typedef std::shared_ptr<BufferHandler> Ptr;

    BufferHandler(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<srv_t>(node,srv_requested,srv_provided)
    {
    }

    virtual bool update(typename srv_t::Request& req, typename srv_t::Response& res)
    {
        res.resp = BufferHandler::updateBuffer(req.group_name,req.data_name,req.value);
        return res.resp;
    }

    virtual bool sync()
    {
        if(sync_mtx_.try_lock())
        {
            buffer_.sync();
            sync_mtx_.unlock();
        }
    }

protected:

    bool updateBuffer(const std::string& group_name, const std::string& data_name, const decltype(srv_t::Request::value)& value)
    {
        sync_mtx_.lock();
        buffer_.update(group_name,data_name,value);
        sync_mtx_.unlock();
        // FIXME add a proper error handling
        return true;
    }

    void add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, srv_t& srv)
    {
        if(this->client_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            this->client_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
            else
                buffer_.add(group_name,data_name,data_ptr);
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

    Buffer<data_t> buffer_;
    std::mutex sync_mtx_;
};

template<class srv_t, typename ...Args>
class CallbackHandler : public InterfaceHandler<srv_t>
{

public:

    typedef std::shared_ptr<CallbackHandler> Ptr;

    using funct_t = std::function<bool(Args...)>;

    CallbackHandler(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
        :InterfaceHandler<srv_t>(node,srv_requested,srv_provided)
    {
    }

    //virtual bool update(typename srv_t::Request& req, typename srv_t::Response& res)
    //{
    //    res.resp = CallbackManager::callback(req.value);
    //    return res.resp;
    //}

    //template<class Q = srv_t>
    //typename std::enable_if<std::is_same<Q, rt_gui::Void>::value, bool>::type
    //update(typename srv_t::Request& req, typename srv_t::Response& res)
    //{
    //    res.resp = fun_();
    //    return res.resp;
    //}
    //
    //template<class Q = srv_t>
    //typename std::enable_if<!std::is_same<Q, rt_gui::Void>::value, bool>::type
    //update(typename srv_t::Request& req, typename srv_t::Response& res)
    //{
    //    res.resp = fun_(req.value);
    //    return res.resp;
    //}

    virtual bool update(typename srv_t::Request& req, typename srv_t::Response& res)
    {
        if constexpr(std::is_same<srv_t,rt_gui::Void>::value)
            res.resp = fun_();
        else
            res.resp = fun_(req.value);
        return res.resp;
    }

    virtual bool sync() // Purely async at the moment
    {
    }

    void add(const std::string& group_name, const std::string& data_name, funct_t fun)
    {
        assert(fun);
        fun_ = fun;
        srv_t srv;
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

protected:

    funct_t fun_;
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class TriggerHandler : public CallbackHandler<rt_gui::Void>
{

public:

    typedef std::shared_ptr<TriggerHandler> Ptr;

    TriggerHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :CallbackHandler<rt_gui::Void>(node,srv_requested,srv_provided)
    {
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntHandler : public BufferHandler<rt_gui::Int,int>
{

public:

    typedef std::shared_ptr<IntHandler> Ptr;

    IntHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :BufferHandler<rt_gui::Int,int>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr)
    {
        rt_gui::Int srv;
        srv.request.min = min;
        srv.request.max = max;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        BufferHandler::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleHandler : public BufferHandler<rt_gui::Double,double>
{

public:

    typedef std::shared_ptr<DoubleHandler> Ptr;

    DoubleHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :BufferHandler<rt_gui::Double,double>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
    {
        rt_gui::Double srv;
        srv.request.min = min;
        srv.request.max = max;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        BufferHandler::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class BoolHandler : public BufferHandler<rt_gui::Bool,bool>
{

public:

    typedef std::shared_ptr<BoolHandler> Ptr;

    BoolHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :BufferHandler<rt_gui::Bool,bool>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, bool* data_ptr)
    {
        rt_gui::Bool srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        BufferHandler::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ListHandler : public BufferHandler<rt_gui::List,std::string>
{

public:

    typedef std::shared_ptr<ListHandler> Ptr;

    ListHandler(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :BufferHandler<rt_gui::List,std::string>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr)
    {
        rt_gui::List srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        for(unsigned int i=0;i<list.size();i++)
            srv.request.list.push_back(list[i]);
        BufferHandler::add(group_name,data_name,data_ptr,srv);
    }

};



} // namespace


#endif
