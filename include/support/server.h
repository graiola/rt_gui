#ifndef SUPPORT_SERVER_H
#define SUPPORT_SERVER_H

#include <support/common.h>
#include <qt_utils/window.h>

namespace rt_gui
{

template<typename data_t, typename data_srv_request_t>
class ServerManagerBase
{

public:

  typedef std::shared_ptr<ServerManagerBase> Ptr;

  ServerManagerBase(std::shared_ptr<Window> window, ros::NodeHandle& node, std::string srv_requested, std::string srv_provided)
  {
     update_ = node.serviceClient<data_srv_request_t>("/" RT_GUI_CLIENT_NAME "/"+srv_requested);
     window_ = window;
  }

  bool update(data_srv_request_t& srv)
  {
    if(update_.exists())
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
    data_srv_request_t srv;
    srv.request.data_name  = data_name;
    srv.request.group_name = group_name;
    srv.request.value = value;
    return update(srv);
  }

protected:
  ros::ServiceServer add_;
  ros::ServiceClient update_;
  std::shared_ptr<Window> window_;

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class SliderServerManager : public QObject, ServerManagerBase<double,rt_gui::updateSlider>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<SliderServerManager> Ptr;

  SliderServerManager(std::shared_ptr<Window> window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ServerManagerBase<double,rt_gui::updateSlider>(window,node,srv_requested,srv_provided)
  {
     add_ = node.advertiseService(srv_provided, &SliderServerManager::addSlider, this); // FIXME to be moved in base


     QObject::connect(this,          SIGNAL(addSlider(const QString&, const QString&, const double&, const double&, const double&)),
                      window_.get(), SLOT(addSlider(const QString&, const QString&, const double&, const double&, const double&)));

     QObject::connect(window_.get(), SIGNAL(updateSlider(QString, QString, double)),
                      this,          SLOT(updateSlider(QString, QString, double)));
  }

  bool addSlider(addSlider::Request  &req, addSlider::Response &res)
  {
    emit addSlider(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.min,req.max,req.init);
    // FIXME add a proper error handling
    res.resp = true;
    return res.resp;
  }

public slots:
  bool updateSlider(QString group_name, QString data_name, double value)
  {

    return update(group_name.toStdString(),data_name.toStdString(),value);
  }

signals:
  void addSlider(const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RadioButtonServerManager : public QObject, ServerManagerBase<bool,rt_gui::updateRadioButton>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<RadioButtonServerManager> Ptr;

  RadioButtonServerManager(std::shared_ptr<Window> window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ServerManagerBase<bool,rt_gui::updateRadioButton>(window,node,srv_requested,srv_provided)
  {
     add_ = node.advertiseService(srv_provided, &RadioButtonServerManager::addRadioButton, this); // FIXME to be moved in base


     QObject::connect(this,          SIGNAL(addRadioButton(const QString&, const QString&, const bool&)),
                      window_.get(), SLOT(addRadioButton(const QString&, const QString&, const bool&)));

     QObject::connect(window_.get(), SIGNAL(updateRadioButton(QString, QString, bool)),
                      this,          SLOT(updateRadioButton(QString, QString, bool)));
  }

  bool addRadioButton(addRadioButton::Request  &req, addRadioButton::Response &res)
  {
    emit addRadioButton(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),req.init);
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
class ComboBoxServerManager : public QObject, ServerManagerBase<std::string,rt_gui::updateComboBox>
{

  Q_OBJECT

public:

  typedef std::shared_ptr<ComboBoxServerManager> Ptr;

  ComboBoxServerManager(std::shared_ptr<Window> window, ros::NodeHandle& node,  std::string srv_requested, std::string srv_provided)
    :ServerManagerBase<std::string,rt_gui::updateComboBox>(window,node,srv_requested,srv_provided)
  {
     add_ = node.advertiseService(srv_provided, &ComboBoxServerManager::addComboBox, this); // FIXME to be moved in base


     QObject::connect(this,          SIGNAL(addComboBox(const QString&, const QString&, const QStringList&, const QString&)),
                      window_.get(), SLOT(addComboBox(const QString&, const QString&, const QStringList&, const QString&)));

     QObject::connect(window_.get(), SIGNAL(updateComboBox(QString, QString, QString)),
                      this,          SLOT(updateComboBox(QString, QString, QString)));
  }

  bool addComboBox(addComboBox::Request  &req, addComboBox::Response &res)
  {
    QStringList list;
    for(unsigned int i=0;i<req.list.size();i++)
      list.push_back(QString::fromStdString(req.list[i]));
    emit addComboBox(QString::fromStdString(req.group_name),QString::fromStdString(req.data_name),list,QString::fromStdString(req.init));
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
