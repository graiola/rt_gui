cmake_minimum_required(VERSION 3.10.2)
project(rt_gui_ros2)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 nor C++17 support. Please use a different C++ compiler.")
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rt_gui_msgs REQUIRED)

# Install Python modules
#ament_python_install_package(bindings/${PROJECT_NAME})

include_directories(
    include
    ${rclpy_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rt_gui_msgs_INCLUDE_DIRS}
    ${rt_gui_core_INCLUDE_DIRS}
)

ament_export_include_directories(include)

ament_package()

## Compile server lib
#add_library(server include/rt_gui/rt_gui_server.h include/rt_gui/support/server.h src/support/server.cpp)
#target_link_libraries(server qt_utils ${LIBRARIES})
#rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")
