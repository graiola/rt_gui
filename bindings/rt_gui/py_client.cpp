#include <pybind11/pybind11.h>
#include <rt_gui/rt_gui_client.h>

namespace py = pybind11;

struct Int
{
    int data;
};


PYBIND11_MODULE(py_client, m) {

    // optional module docstring
    m.doc() = "pybind11 rt_gui_client plugin";

    py::class_<Int>(m, "Int")  //
            .def_readwrite("data", &Int::data);

    py::class_<rt_gui::RtGuiClient>(m,"RtGuiClient")
        .def("istance", &rt_gui::RtGuiClient::getIstance, py::return_value_policy::reference)
        .def("addInt", [](const std::string& group_name, const std::string& data_name, const int& min, const int& max, int* data_ptr, bool sync = true, bool load_init_from_server = false) {
        rt_gui::RtGuiClient::getIstance().addInt(group_name,data_name,min,max,data_ptr,sync,load_init_from_server);
    });
}
