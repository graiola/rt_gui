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

find_package(Qt5Core ${MINIMUM_QT_VERSION} REQUIRED)

find_package(catkin QUIET)

if (${catkin_FOUND})

    find_package(catkin REQUIRED roscpp rospy rt_gui_msgs rt_gui_core)

    catkin_python_setup()

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}_server
      CATKIN_DEPENDS rt_gui_msgs rt_gui_core
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )

    ## Compile server lib
    add_library(${PROJECT_NAME}_server
        include/${PROJECT_NAME}/support/ros_node.h
        include/${PROJECT_NAME}/support/server.h
        include/${PROJECT_NAME}/rt_gui_server.h
        src/support/server.cpp)
    target_link_libraries(${PROJECT_NAME}_server PUBLIC ${catkin_LIBRARIES} Qt5::Core)

    ## Compile the server node
    add_executable(${PROJECT_NAME}_server_node src/server_node.cpp)
    target_link_libraries(${PROJECT_NAME}_server_node PRIVATE ${PROJECT_NAME}_server)

    ## Bindings
    #find_package(PythonInterp REQUIRED) #find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
    #find_package(pybind11 QUIET)
    #message(STATUS "check for pybind11")
    #if(${pybind11_FOUND})
    #    message(STATUS "compiling python bindings")
    #    # Server
    #    pybind11_add_module(py_server bindings/rt_gui/py_server.cpp)
    #    target_link_libraries(py_server PRIVATE server)
    #    install(TARGETS py_server
    #            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
    #    # Client
    #    pybind11_add_module(py_client bindings/rt_gui/py_client.cpp)
    #    target_link_libraries(py_client PRIVATE qt_utils ${catkin_LIBRARIES})
    #    install(TARGETS py_client
    #            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
    #    # Tests
    #    catkin_install_python(PROGRAMS bindings/rt_gui/test_server.py bindings/rt_gui/test_client.py
    #                          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    #else()
    #    message(STATUS "pybind not found")
    #endif()

    ## Compile test
    #if (CATKIN_ENABLE_TESTING)
    #    add_executable(test_qt test/test_qt.cpp)
    #    target_link_libraries(test_qt PRIVATE qt_utils)
    #
    #    add_executable(test_server include/rt_gui/rt_gui_server.h test/test_server.cpp)
    #    add_dependencies(test_server ${rt_gui_EXPORTED_TARGETS})
    #    target_link_libraries(test_server PRIVATE server)
    #
    #    add_executable(test_client include/rt_gui/rt_gui_client.h test/test_client.cpp)
    #    add_dependencies(test_client ${rt_gui_EXPORTED_TARGETS})
    #    target_link_libraries(test_client PRIVATE qt_utils ${LIBRARIES})
    #
    #    add_executable(test_client_1 include/rt_gui/rt_gui_client.h test/test_client_1.cpp)
    #    add_dependencies(test_client_1 ${rt_gui_EXPORTED_TARGETS})
    #    target_link_libraries(test_client_1 PRIVATE qt_utils ${LIBRARIES})
    #
    #    add_executable(test_console test/test_console.cpp)
    #    target_link_libraries(test_console PRIVATE console)
    #endif()

    ## Install
    install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_server_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

    install(DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      PATTERN ".svn" EXCLUDE)

else()
  message("catkin not found, skipping ${PROJECT_NAME} package")
endif()
