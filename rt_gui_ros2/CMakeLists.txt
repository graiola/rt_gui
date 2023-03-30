cmake_minimum_required(VERSION 3.5)
project(rt_gui_ros2)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized -fPIC")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized -fPIC")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 nor C++17 support. Please use a different C++ compiler.")
endif()

option(QT_VERSION_GREATER_THAN_5_5 "Set to true if Qt5 > 5.5 is installed" FALSE)

set(CMAKE_AUTOMOC ON)

if(${QT_VERSION_GREATER_THAN_5_5})
    set(MINIMUM_QT_VERSION 5.9)
else()
    set(MINIMUM_QT_VERSION 5.5)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(Qt5Core ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)

find_package(ament_cmake_python QUIET)

if (${ament_cmake_python_FOUND})

    find_package(rclpy REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rt_gui_core REQUIRED)
    find_package(rt_gui_msgs REQUIRED)

    # Install Python modules
    #ament_python_install_package(bindings/${PROJECT_NAME})

    include_directories(
        include
        ${rclpy_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rt_gui_core_INCLUDE_DIRS}
        ${rt_gui_msgs_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
    )

    ament_export_include_directories(include)

    ament_package()

    message("----------------------------")
    message(${rt_gui_core_INCLUDE_DIRS})

    ## Compile server lib
    add_library(${PROJECT_NAME}_server
        include/${PROJECT_NAME}/support/ros_node.h
        include/${PROJECT_NAME}/support/server.h
        include/${PROJECT_NAME}/rt_gui_server.h
        src/support/server.cpp)
    target_link_libraries(${PROJECT_NAME}_server PUBLIC ${rclcpp_LIBRARIES} ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} Qt5::Core Qt5::Widgets)

    ## Compile server lib
    #add_library(server include/rt_gui_ros2/rt_gui_server.h include/rt_gui/support/server.h src/support/server.cpp)
    #target_link_libraries(server qt_utils ${LIBRARIES})
    #rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")

    #add_executable(test_rosnode test/test_rosnode.cpp)
    #target_link_libraries(test_rosnode ${rclcpp_LIBRARIES})

else()
  message("ament not found, skipping ${PROJECT_NAME} package")
endif()
