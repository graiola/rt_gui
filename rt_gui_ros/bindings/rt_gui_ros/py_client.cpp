#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <rt_gui_ros/rt_gui_client.h>

namespace py = pybind11;

PYBIND11_MODULE(rt_gui_ros_client, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui_client plugin";

    py::class_<rt_gui::RtGuiClient>(m,"RtGuiClient")
        .def_static("init", []()
        {
          rt_gui::RtGuiClient::getIstance().init();
        })
        .def_static("init", [](const std::string& server_name)
        {
          rt_gui::RtGuiClient::getIstance().init(server_name);
        })
        .def_static("init", [](const std::string& server_name, const std::string& client_name)
        {
          rt_gui::RtGuiClient::getIstance().init(server_name,client_name);
        })
        .def_static("init", [](const std::string& server_name, const std::string& client_name, const double& timeout)
        {
          rt_gui::RtGuiClient::getIstance().init(server_name,client_name,ros::Duration(timeout));
        })
        .def_static("addInt", [](const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addInt(group_name,data_name,min,max,fun,sync);
        })
        .def_static("addDouble", [](const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(double)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addDouble(group_name,data_name,min,max,fun,sync);
        })
        .def_static("addBool", [](const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addBool(group_name,data_name,fun,sync);
        })
        .def_static("addTrigger", [](const std::string& group_name, const std::string& data_name, std::function<void()> fun)
        {
          rt_gui::RtGuiClient::getIstance().addTrigger(group_name,data_name,fun);
        })
        .def_static("addList", [](const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addList(group_name,data_name,list,fun,sync);
        })
        .def_static("addList", [](const std::string& group_name, const std::string& data_name, const std::string& init_value, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addList(group_name,data_name,init_value,list,fun,sync);
        })
        .def_static("remove", [](const std::string& group_name, const std::string& data_name)
        {
          rt_gui::RtGuiClient::getIstance().remove(group_name,data_name);
        })
        .def_static("sync", []()
        {
          rt_gui::RtGuiClient::getIstance().sync();
        });

}
