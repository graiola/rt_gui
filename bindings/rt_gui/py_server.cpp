#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rt_gui/rt_gui_server.h>

namespace py = pybind11;

PYBIND11_MODULE(py_server, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui plugin";

    py::class_<QWidget>(m, "QWidget")
      .def(py::init<>());

    py::class_<rt_gui::RosServerNode>(m,"RosServerNode")
        .def(py::init<>())
        .def("spawn", &rt_gui::RosServerNode::spawn, py::arg("ros_node_name") = RT_GUI_SERVER_NAME,
             py::arg("parent") = py::none());

    py::class_<rt_gui::RtGuiServer>(m,"RtGuiServer")
        .def("istance", &rt_gui::RtGuiServer::getIstance, py::return_value_policy::reference)
        //.def("run", &rt_gui::RtGuiServer::run);
        .def("run", [](const std::string& name=RT_GUI_SERVER_NAME) {
        int argc = 1;
        char* argv[] = {strdup("py_server"), strdup(name.c_str())};
        return rt_gui::RtGuiServer::getIstance().run(argc, argv);
    });
}
