#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <rt_gui/rt_gui_client.h>

namespace py = pybind11;

PYBIND11_MODULE(py_client, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui_client plugin";

    py::class_<rt_gui::RtGuiClient>(m,"RtGuiClient")
        .def("istance", &rt_gui::RtGuiClient::getIstance, py::return_value_policy::reference)
        .def("init", [](const std::string& ros_namespace)
        {
          rt_gui::RtGuiClient::getIstance().init(ros_namespace);
        })
        .def("addInt", [](const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(int)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addInt(group_name,data_name,min,max,fun,sync);
        })
        .def("addDouble", [](const std::string& group_name, const std::string& data_name, const int& min, const int& max, std::function<void(double)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addDouble(group_name,data_name,min,max,fun,sync);
        })
        .def("addBool", [](const std::string& group_name, const std::string& data_name, std::function<void(bool)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addBool(group_name,data_name,fun,sync);
        })
        .def("addTrigger", [](const std::string& group_name, const std::string& data_name, std::function<void()> fun)
        {
          rt_gui::RtGuiClient::getIstance().addTrigger(group_name,data_name,fun);
        })
        .def("addList", [](const std::string& group_name, const std::string& data_name, const std::vector<std::string>& list, std::function<void(std::string)> fun, bool sync = true)
        {
          rt_gui::RtGuiClient::getIstance().addList(group_name,data_name,list,fun,sync);
        })
        .def("remove", [](const std::string& group_name, const std::string& data_name)
        {
          rt_gui::RtGuiClient::getIstance().remove(group_name,data_name);
        })
        .def("sync", []()
        {
          rt_gui::RtGuiClient::getIstance().sync();
        });

}
