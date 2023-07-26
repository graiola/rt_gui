#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rt_gui_ros/rt_gui_server.h>
#include <csignal>

namespace py = pybind11;

void catch_signals() {
  auto handler = [](int code) { throw std::runtime_error("SIGNAL " + std::to_string(code)); };
  signal(SIGINT,  handler);
  signal(SIGTERM, handler);
  signal(SIGKILL, handler);
}

PYBIND11_MODULE(rt_gui_ros_server, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui_server plugin";

    py::class_<QWidget>(m, "QWidget")
      .def(py::init<>());

    //py::class_<rt_gui::RosServerNode>(m,"RosServerNode")
    //    .def(py::init<>())
    //    .def("init", &rt_gui::RosServerNode::init, py::arg("server_name") = RT_GUI_SERVER_NAME,
    //         py::arg("parent") = py::none());

    py::class_<rt_gui::RtGuiServer>(m,"RtGuiServer")
        .def_static("run", [](const std::string& server_name) {
        catch_signals();
        int argc = 2;
        char* argv[] = {strdup("rt_gui_ros_server"), strdup(server_name.c_str())};
        return rt_gui::RtGuiServer::getIstance().run(argc, argv);
    })
        .def_static("run", []() {
        catch_signals();
        int argc = 2;
        char* argv[] = {strdup("rt_gui_ros_server"), strdup(RT_GUI_SERVER_NAME)};
        return rt_gui::RtGuiServer::getIstance().run(argc, argv);
    })
        .def_static("stop", []() {
        return rt_gui::RtGuiServer::getIstance().stop();
    });
}
