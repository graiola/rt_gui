#include <pybind11/pybind11.h>
#include <rt_gui/support/server.h>

namespace py = pybind11;


PYBIND11_MODULE(py_server, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui plugin";

    py::class_<rt_gui::RosServerNode>(m,"RosServerNode")
        .def(py::init<>())
        .def("spawn", &rt_gui::RosServerNode::spawn, py::arg("ros_node_name") = "test",
             py::arg("parent") = py::none());
}
