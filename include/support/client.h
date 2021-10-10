#ifndef SUPPORT_CLIENT_H
#define SUPPORT_CLIENT_H

#include <support/common.h>
#include <mutex>

namespace rt_gui
{

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

template<typename data_t, class srv_t>
class ClientManagerBase
{

public:

    typedef std::shared_ptr<ClientManagerBase> Ptr;

    ClientManagerBase(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
    {
        update_ = node.advertiseService(srv_provided, &ClientManagerBase::update, this);
        add_    = node.serviceClient<srv_t>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
    }

    bool update(typename srv_t::Request& req, typename srv_t::Response& res)
    {
        res.resp = ClientManagerBase::updateBuffer(req.group_name,req.data_name,req.value);
        return res.resp;
    }

    bool updateBuffer(const std::string& group_name, const std::string& data_name, const decltype(srv_t::Request::value)& value)
    {
        sync_mtx_.lock();
        buffer_.update(group_name,data_name,value);
        sync_mtx_.unlock();
        // FIXME add a proper error handling
        return true;
    }

    bool sync()
    {
        if(sync_mtx_.try_lock())
        {
            buffer_.sync();
            sync_mtx_.unlock();
        }
    }

protected:

    void add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, srv_t& srv)
    {
        if(add_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            add_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
            else
                buffer_.add(group_name,data_name,data_ptr);
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

    ros::ServiceServer update_;
    ros::ServiceClient add_;
    std::mutex sync_mtx_;
    Buffer<data_t> buffer_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ButtonClientManager
{

public:

    typedef std::shared_ptr<ButtonClientManager> Ptr;

    typedef std::function<bool ()> funct_t;

    ButtonClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    {
        update_ = node.advertiseService(srv_provided, &ButtonClientManager::update, this);
        add_    = node.serviceClient<rt_gui::Void>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
    }

    bool update(Void::Request& req, Void::Response& res)
    {
        res.resp = fun_(); // Trigger
        return res.resp;
    }

    void add(const std::string& group_name, const std::string& data_name, funct_t fun)
    {

        assert(fun);
        fun_ = fun;
        rt_gui::Void srv;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        if(add_.waitForExistence(ros::Duration(_ros_services.wait_service_secs)))
        {
            add_.call(srv);
            if(srv.response.resp == false)
                throw std::runtime_error("RtGuiServer::add::resp is false!");
        }
        else
            throw std::runtime_error("RtGuiServer::add service is not available!");
    }

private:

    ros::ServiceServer update_;
    ros::ServiceClient add_;

    funct_t fun_;

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class IntSliderClientManager : public ClientManagerBase<int,rt_gui::Int>
{

public:

    typedef std::shared_ptr<IntSliderClientManager> Ptr;

    IntSliderClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :ClientManagerBase<int,rt_gui::Int>(node,srv_requested,srv_provided)
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
        ClientManagerBase::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DoubleSliderClientManager : public ClientManagerBase<double,rt_gui::Double>
{

public:

    typedef std::shared_ptr<DoubleSliderClientManager> Ptr;

    DoubleSliderClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :ClientManagerBase<double,rt_gui::Double>(node,srv_requested,srv_provided)
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
        ClientManagerBase::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RadioButtonClientManager : public ClientManagerBase<bool,rt_gui::Bool>
{

public:

    typedef std::shared_ptr<RadioButtonClientManager> Ptr;

    RadioButtonClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :ClientManagerBase<bool,rt_gui::Bool>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, bool* data_ptr)
    {
        rt_gui::Bool srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        ClientManagerBase::add(group_name,data_name,data_ptr,srv);
    }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ComboBoxClientManager : public ClientManagerBase<std::string,rt_gui::String>
{

public:

    typedef std::shared_ptr<ComboBoxClientManager> Ptr;

    ComboBoxClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
        :ClientManagerBase<std::string,rt_gui::String>(node,srv_requested,srv_provided)
    {
    }

    void add(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr)
    {
        rt_gui::String srv;
        srv.request.value = *data_ptr;
        srv.request.group_name = group_name;
        srv.request.data_name = data_name;
        for(unsigned int i=0;i<list.size();i++)
            srv.request.list.push_back(list[i]);
        ClientManagerBase::add(group_name,data_name,data_ptr,srv);
    }

};



} // namespace


#endif
