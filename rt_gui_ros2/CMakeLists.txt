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

    ## Compile server lib
    add_library(${PROJECT_NAME}_server SHARED
        include/${PROJECT_NAME}/support/ros_node.h
        include/${PROJECT_NAME}/support/server.h
        include/${PROJECT_NAME}/rt_gui_server.h
        src/support/server.cpp)
    target_link_libraries(${PROJECT_NAME}_server PUBLIC ${rclcpp_LIBRARIES} ${rt_gui_core_LIBRARIES} ${rt_gui_msgs_LIBRARIES} Qt5::Core Qt5::Widgets)

    ## Compile the server node
    add_executable(${PROJECT_NAME}_server_node src/server_node.cpp)
    target_link_libraries(${PROJECT_NAME}_server_node PRIVATE ${PROJECT_NAME}_server)


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

    install(
      TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_server_node
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

else()
  message("ament not found, skipping ${PROJECT_NAME} package")
endif()
