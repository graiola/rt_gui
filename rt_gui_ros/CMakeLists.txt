cmake_minimum_required(VERSION 3.5)
project(rt_gui_ros)

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

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

find_package(rt_gui_msgs REQUIRED)
find_package(rt_gui_core REQUIRED)
find_package(Qt5Core ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)

if (${catkin_FOUND})

    set(ROS_VERSION "ros")
    add_definitions(-DROS)
    message(STATUS "Add ROS definition")

    find_package(rospy REQUIRED)
    find_package(roscpp REQUIRED)

    set(ROS_LIB ${roscpp_LIBRARIES})

    configure_file(setup.py.${ROS_VERSION} ${CMAKE_SOURCE_DIR}/setup.py)

    catkin_python_setup()

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}_server
      CATKIN_DEPENDS rt_gui_msgs rt_gui_core
    )

  elseif (${ament_cmake_FOUND})

      set(ROS_VERSION "ros2")
      add_definitions(-DROS2)
      message(STATUS "Add ROS2 definition")

      find_package(rclpy REQUIRED)
      find_package(rclcpp REQUIRED)

      set(ROS_LIB ${rclcpp_LIBRARIES})

      configure_file(setup.py.${ROS_VERSION} ${CMAKE_SOURCE_DIR}/setup.py)

      ament_python_install_package(bindings)

      ament_export_dependencies(
        rt_gui_core
        rt_gui_msgs
      )
      ament_export_include_directories(include)
      ament_export_libraries(${PROJECT_NAME}_server)
      ament_package()

  endif()

  include_directories(
      include/${ROS_VERSION}
      ${rt_gui_msgs_INCLUDE_DIRS}
      ${rt_gui_core_INCLUDE_DIRS}
      ${Qt5Core_INCLUDE_DIRS}
      ${Qt5Widgets_INCLUDE_DIRS}
  )

## Compile server lib
add_library(${PROJECT_NAME}_server SHARED
    include/${ROS_VERSION}/${PROJECT_NAME}/support/ros_node.h
    include/${ROS_VERSION}/${PROJECT_NAME}/support/server.h
    include/${ROS_VERSION}/${PROJECT_NAME}/rt_gui_server.h
    src/${ROS_VERSION}/support/server.cpp)
target_link_libraries(${PROJECT_NAME}_server PUBLIC ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES} Qt5::Core Qt5::Widgets ${ROS_LIB})

## Compile the server node
add_executable(${PROJECT_NAME}_server_node src/server_node.cpp)
target_link_libraries(${PROJECT_NAME}_server_node PRIVATE ${PROJECT_NAME}_server)

## Compile the tests
add_executable(test_server test/test_server.cpp)
target_link_libraries(test_server PRIVATE ${PROJECT_NAME}_server)

add_executable(test_client test/${ROS_VERSION}/test_client.cpp)
target_link_libraries(test_client PRIVATE ${ROS_LIB} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

add_executable(test_client_1 test/${ROS_VERSION}/test_client_1.cpp)
target_link_libraries(test_client_1 PRIVATE ${ROS_LIB} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

## Bindings
find_package(pybind11 QUIET)
message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")
    # Server
    pybind11_add_module(py_server bindings/${PROJECT_NAME}/py_server.cpp)
    target_link_libraries(py_server PRIVATE ${PROJECT_NAME}_server)

    # Client
    pybind11_add_module(py_client bindings/${PROJECT_NAME}/py_client.cpp)
    target_link_libraries(py_client PRIVATE ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES})

    install(TARGETS py_server py_client
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
    )

else()
    message(STATUS "pybind not found")
endif()

install(
  DIRECTORY include/${ROS_VERSION}/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(TARGETS ${PROJECT_NAME}_server_node test_client test_client_1
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_server
       EXPORT export_${PROJECT_NAME}_server
       DESTINATION lib)

install(PROGRAMS
  bindings/${PROJECT_NAME}/test_client.py
  bindings/${PROJECT_NAME}/test_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
 DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)
