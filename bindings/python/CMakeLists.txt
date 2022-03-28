cmake_minimum_required(VERSION 3.5)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0") 
    message("Please consider to switch to latest version to enable 
    more reliable Python3 binaries detection")
else()
    #find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
    find_package(PythonInterp REQUIRED)
endif()

find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(py_server pyRtGuiServer.cpp)
    target_link_libraries(py_server PUBLIC server)

    install(TARGETS py_server
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
else()
    message(STATUS "pybind not found")
endif()

