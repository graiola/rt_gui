#ifndef SUPPORT_CLIENT_H
#define SUPPORT_CLIENT_H

#include <support/common.h>
#include <mutex>

namespace rt_gui
{

template<typename data_t, typename data_srv_request_t>
class ClientManagerBase
{

public:

  typedef std::pair<std::string,std::string> buffer_key_t;
  typedef std::pair<data_t*,data_t> buffer_value_t;
  typedef std::map<buffer_key_t, buffer_value_t> buffer_t;

  typedef std::shared_ptr<ClientManagerBase> Ptr;

  ClientManagerBase(ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
  {
     add_ = node.serviceClient<data_srv_request_t>("/" RT_GUI_SERVER_NAME "/"+srv_requested);
  }

  bool update(const std::string& group_name, const std::string& data_name, const data_t& value)
  {
     sync_mtx_.lock();
     buffer_[buffer_key_t(group_name,data_name)].second = value;
     sync_mtx_.unlock();
     // FIXME add a proper error handling
     return true;
  }

  bool sync()
  {
    if(sync_mtx_.try_lock())
    {
      for(auto tmp_map : buffer_)
        if(tmp_map.second.first!=nullptr) // The data pointer still exists
          *tmp_map.second.first = tmp_map.second.second;
      sync_mtx_.unlock();
    }
  }

protected:

  void add(const std::string& group_name, const std::string& data_name, data_t* data_ptr, data_srv_request_t& srv)
  {
    assert(data_ptr);
    if(add_.exists())
    {
      add_.call(srv);
      if(srv.response.resp == false)
        throw std::runtime_error("RtGuiServer::add::resp is false!");
      else
        buffer_[buffer_key_t(group_name,data_name)] = buffer_value_t(data_ptr,*data_ptr);
    }
    else
    {
      throw std::runtime_error("RtGuiServer::add service is not available!");
    }
  }

  ros::ServiceServer update_;
  ros::ServiceClient add_;
  buffer_t buffer_;
  std::mutex sync_mtx_;

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SliderClientManager : public ClientManagerBase<double,rt_gui::addSlider>
{

public:

  typedef std::shared_ptr<SliderClientManager> Ptr;

  SliderClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ClientManagerBase<double,rt_gui::addSlider>(node,srv_requested,srv_provided)
  {
     update_ = node.advertiseService(srv_provided, &SliderClientManager::update, this); // FIXME to be moved in base
  }

  bool update(updateSlider::Request& req, updateSlider::Response& res)
  {
    res.resp = ClientManagerBase::update(req.group_name,req.data_name,req.value);
    return res.resp;
  }

  void add(const std::string& group_name, const std::string& data_name, const double& min, const double& max, double* data_ptr)
  {
    rt_gui::addSlider srv;
    srv.request.min = min;
    srv.request.max = max;
    srv.request.init = *data_ptr;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    ClientManagerBase::add(group_name,data_name,data_ptr,srv);
  }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RadioButtonClientManager : public ClientManagerBase<bool,rt_gui::addRadioButton>
{

public:

  typedef std::shared_ptr<RadioButtonClientManager> Ptr;

  RadioButtonClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ClientManagerBase<bool,rt_gui::addRadioButton>(node,srv_requested,srv_provided)
  {
     update_ = node.advertiseService(srv_provided, &RadioButtonClientManager::update, this); // FIXME to be moved in base
  }

  bool update(updateRadioButton::Request& req, updateRadioButton::Response& res)
  {
    res.resp = ClientManagerBase::update(req.group_name,req.data_name,req.value);
    return res.resp;
  }

  void add(const std::string& group_name, const std::string& data_name, bool* data_ptr)
  {
    rt_gui::addRadioButton srv;
    srv.request.init = *data_ptr;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    ClientManagerBase::add(group_name,data_name,data_ptr,srv);
  }

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ComboBoxClientManager : public ClientManagerBase<std::string,rt_gui::addComboBox>
{

public:

  typedef std::shared_ptr<ComboBoxClientManager> Ptr;

  ComboBoxClientManager(ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ClientManagerBase<std::string,rt_gui::addComboBox>(node,srv_requested,srv_provided)
  {
     update_ = node.advertiseService(srv_provided, &ComboBoxClientManager::update, this); // FIXME to be moved in base
  }

  bool update(updateComboBox::Request& req, updateComboBox::Response& res)
  {
    res.resp = ClientManagerBase::update(req.group_name,req.data_name,req.value);
    return res.resp;
  }

  void add(const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::string* data_ptr)
  {
    rt_gui::addComboBox srv;
    srv.request.init = *data_ptr;
    srv.request.group_name = group_name;
    srv.request.data_name = data_name;
    for(unsigned int i=0;i<list.size();i++)
      srv.request.list.push_back(list[i]);
    ClientManagerBase::add(group_name,data_name,data_ptr,srv);
  }

};



} // namespace


#endif
