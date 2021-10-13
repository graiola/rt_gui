#ifndef SUPPORT_COMMON_H
#define SUPPORT_COMMON_H

#include <ros/ros.h>

#include <rt_gui/Bool.h>
#include <rt_gui/Double.h>
#include <rt_gui/Int.h>
#include <rt_gui/List.h>
#include <rt_gui/Void.h>

#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <tuple>

namespace rt_gui
{


typedef std::pair<std::string,std::string> key_t;

template <class data_t>
class CallbackBuffer {

public:

    using callback_t = std::function<void(data_t)>;

    CallbackBuffer() {}
    //                   0       1         2       3
    typedef std::tuple<data_t*,data_t,callback_t,bool> value_t;
    typedef std::map<key_t,value_t> buffer_t;

    bool add(const std::string& key1, const std::string& key2, data_t data, callback_t callback, bool sync = true)
    {
        if(callback!=nullptr)
        {
            buffer_[key_t(key1,key2)] = value_t(nullptr,data,callback,sync);
            return true;
        }
        else
            return false;
    }

    bool add(const std::string& key1, const std::string& key2, data_t* data_ptr, bool sync = true)
    {
        if(data_ptr!=nullptr)
        {
            buffer_[key_t(key1,key2)] = value_t(data_ptr,*data_ptr,nullptr,sync);
            return true;
        }
        else
            return false;
    }

    bool update(const std::string& key1, const std::string& key2, const data_t& value)
    {
        if(std::get<3>(buffer_[key_t(key1,key2)])) // Sync - Copy the new data in the buffer
            std::get<1>(buffer_[key_t(key1,key2)]) = value;
        else // Copy the data directly into the raw data or call the callback
        {
            if (std::get<0>(buffer_[key_t(key1,key2)])!=nullptr) // data pointer still exists
                *std::get<0>(buffer_[key_t(key1,key2)]) = value;
            else if(std::get<2>(buffer_[key_t(key1,key2)])!=nullptr) // callback still exists
                std::get<2>(buffer_[key_t(key1,key2)])(value);
            else
                return false;
        }
        return true;
    }

    bool sync()
    {
        for(auto tmp_map : buffer_)
        {
            if(std::get<3>(tmp_map.second)) // Sync
            {
                if (std::get<0>(tmp_map.second)!=nullptr) // data pointer still exists
                    *std::get<0>(tmp_map.second) = std::get<1>(tmp_map.second);
                else if (std::get<2>(tmp_map.second)!=nullptr) // callback still exists
                    std::get<2>(tmp_map.second)(std::get<1>(tmp_map.second));
                else
                    return false;
            }
        }
        return true;
    }

private:
    buffer_t buffer_;
};

#define RT_GUI_SERVER_NAME "rt_gui_server"
#define RT_GUI_CLIENT_NAME "rt_gui_client"

struct
{
    struct
    {
        std::string add    = "add_double";
        std::string update = "update_double";
    } double_srvs;

    struct
    {
        std::string add    = "add_int";
        std::string update = "update_int";
    } int_srvs;

    struct
    {
        std::string add    = "add_bool";
        std::string update = "update_bool";
    } bool_srvs;

    struct
    {
        std::string add    = "add_list";
        std::string update = "update_list";
    } list_srvs;

    struct
    {
        std::string add    = "add_trigger";
        std::string update = "update_trigger";
    } trigger_srvs;

    unsigned int n_threads = 3;
    double wait_service_secs = 10.0;
    std::string remove_service = "remove_widget";

} _ros_services;

class RosNode
{
public:
    RosNode(const std::string& ros_node_name, const unsigned int& n_threads)
    {
        init(ros_node_name,n_threads);
    }

    RosNode()
    {
        init_ = false;
    }

    void init(const std::string& ros_node_name, const unsigned int& n_threads)
    {
        int argc = 1;
        char* arg0 = strdup(ros_node_name.c_str());
        char* argv[] = {arg0, nullptr};
        ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
        free(arg0);

        if(ros::master::check())
        {
            ros_nh_.reset(new ros::NodeHandle(ros_node_name));
        }
        else
        {
            throw std::runtime_error("roscore not found... Did you start the server?");
        }

        spinner_.reset(new ros::AsyncSpinner(n_threads)); // Use n_threads to keep the ros magic alive
        spinner_->start();

        init_ = true;
    }

    ~RosNode()
    {
        if(init_ == true)
        {
            ros_nh_->shutdown();
            spinner_->stop();
        }
    }

    ros::NodeHandle& getNode()
    {
        if(init_ == true)
            return *ros_nh_.get();
        else
            throw std::runtime_error("RosNode not initialized");
    }

    bool reset()
    {
        if(init_ == true)
        {
            ros_nh_->shutdown();
            spinner_->stop();
            init_ = false;
            return true;
        }
        else
        {
            throw std::runtime_error("RosNode not initialized");
            return false;
        }
    }

    bool initDone()
    {
        return init_;
    }

protected:
    bool init_;
    std::shared_ptr<ros::NodeHandle> ros_nh_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
};

} // namespace

#endif
