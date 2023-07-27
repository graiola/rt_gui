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

find_package(ament_cmake QUIET)

if (${ament_cmake_FOUND})

    find_package(rclpy REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rt_gui_core REQUIRED)
    find_package(rt_gui_msgs REQUIRED)

    include_directories(
        include
        ${rclpy_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rt_gui_core_INCLUDE_DIRS}
        ${rt_gui_msgs_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
    )

    ## Compile server lib
    add_library(${PROJECT_NAME}_server SHARED
        include/${PROJECT_NAME}/support/ros_node.h
        include/${PROJECT_NAME}/support/server.h
        include/${PROJECT_NAME}/rt_gui_server.h
        src/support/server.cpp)
    target_link_libraries(${PROJECT_NAME}_server PUBLIC ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES} Qt5::Core Qt5::Widgets)

    ## Bindings
    #find_package(PythonInterp REQUIRED) #find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
    find_package(pybind11 QUIET)
    message(STATUS "check for pybind11")
    if(${pybind11_FOUND})
        message(STATUS "compiling python bindings")
        # Server
        pybind11_add_module(py_server bindings/${PROJECT_NAME}/py_server.cpp)
        target_link_libraries(py_server PRIVATE ${PROJECT_NAME}_server)

        # Client
        pybind11_add_module(py_client bindings/${PROJECT_NAME}/py_client.cpp)
        target_link_libraries(py_client PRIVATE ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

        # Install
        ament_python_install_package(bindings/${PROJECT_NAME})

        install(TARGETS py_server py_client
          DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
        )

        #install(PROGRAMS
        #  bindings/${PROJECT_NAME}/test_client.py
        #  bindings/${PROJECT_NAME}/test_server.py
        #  DESTINATION lib/${PROJECT_NAME}
        #)

    else()
        message(STATUS "pybind not found")
    endif()

    ## Compile the server node
    add_executable(${PROJECT_NAME}_server_node src/server_node.cpp)
    target_link_libraries(${PROJECT_NAME}_server_node PRIVATE ${PROJECT_NAME}_server)

    ## Compile the test
    add_executable(test_rosnode test/test_rosnode.cpp)
    target_link_libraries(test_rosnode PRIVATE ${PROJECT_NAME}_server)

    add_executable(test_server test/test_server.cpp)
    target_link_libraries(test_server PRIVATE ${PROJECT_NAME}_server)

    add_executable(test_client test/test_client.cpp)
    target_link_libraries(test_client PRIVATE ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

    add_executable(test_client_1 test/test_client_1.cpp)
    target_link_libraries(test_client_1 PRIVATE ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

    ## Export information to downstream packages
    ament_export_dependencies(
      rclcpp
      rt_gui_core
      rt_gui_msgs
    )
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME}_server)
    ament_package()

    ## Install
    install(
      DIRECTORY include/
      DESTINATION include
    )

    install(TARGETS ${PROJECT_NAME}_server_node test_client test_client_1
           DESTINATION lib/${PROJECT_NAME})

    install(TARGETS ${PROJECT_NAME}_server
           EXPORT export_${PROJECT_NAME}_server
           DESTINATION lib)

else()
  message("ament not found, skipping ${PROJECT_NAME} package")
endif()
