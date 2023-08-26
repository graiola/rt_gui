cmake_minimum_required(VERSION 3.10.2)
project(rt_gui)

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

option(QT_VERSION_GREATER_THAN_5_5 "Set to true if Qt5 > 5.5 is installed" FALSE)

set(CMAKE_AUTOMOC ON)

if(${QT_VERSION_GREATER_THAN_5_5})
    set(MINIMUM_QT_VERSION 5.9)
else()
    set(MINIMUM_QT_VERSION 5.5)
endif()

## Get custom path to QT from env
message(STATUS "Searching QT in '${QT_PATH}'")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Get CMAKE_PREFIX_PATH from environment, and append QT_PATH
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
string(REPLACE ":" ";" CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})

find_package(Qt5Widgets ${MINIMUM_QT_VERSION} QUIET)
find_package(Qwt ${MINIMUM_QT_VERSION} QUIET)
find_package(catkin REQUIRED roscpp rospy std_msgs message_generation)

#catkin_python_setup()

add_service_files(
  FILES
  Bool.srv
  Double.srv
  Int.srv
  List.srv
  Check.srv
  Void.srv
  Text.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qt_utils server
  CATKIN_DEPENDS pybind11_catkin
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIRS}
)

if (${QWT_FOUND} AND ${Qt5Widgets_FOUND})

    message(STATUS "Compiling QT related code")

    ## Compile qt_utils lib
    include(src/qt_utils/qt_utils.cmake)

    ## Compile console lib
    add_library(console src/console/console_core.cpp src/console/console.cpp)
    target_link_libraries(console readline pthread)

    ## Compile server lib
    add_library(server include/rt_gui/rt_gui_server.h include/rt_gui/support/server.h src/support/server.cpp)
    target_link_libraries(server qt_utils ${catkin_LIBRARIES})

    ## Compile the server node
    add_executable(server_node src/server_node.cpp)
    add_dependencies(server_node ${rt_gui_EXPORTED_TARGETS})
    target_link_libraries(server_node PRIVATE server)

    ## Bindings
    #find_package(PythonInterp REQUIRED) #find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
    find_package(pybind11_catkin QUIET)
    message(STATUS "check for pybind11_catkin")
    if(${pybind11_catkin_FOUND})
        message(STATUS "compiling python bindings")
        # Server
        pybind_add_module(rt_gui_server_py bindings/rt_gui/py_server.cpp)
        target_link_libraries(rt_gui_server_py PRIVATE server)

        # Client
        pybind_add_module(rt_gui_client_py bindings/rt_gui/py_client.cpp)
        target_link_libraries(rt_gui_client_py PRIVATE qt_utils ${catkin_LIBRARIES})

        set_target_properties(rt_gui_server_py PROPERTIES
                PREFIX ""
                LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                )

        set_target_properties(rt_gui_client_py PROPERTIES
                PREFIX ""
                LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                )

        install(TARGETS rt_gui_server_py
                        rt_gui_client_py
                        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

        # Tests
        catkin_install_python(PROGRAMS bindings/rt_gui/test_server.py bindings/rt_gui/test_client.py
                              DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    else()
        message(STATUS "pybind not found")
    endif()

    install(TARGETS server_node qt_utils server
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    ## Compile test
    if (CATKIN_ENABLE_TESTING)
        add_executable(test_qt test/test_qt.cpp)
        target_link_libraries(test_qt PRIVATE qt_utils)

        add_executable(test_server include/rt_gui/rt_gui_server.h test/test_server.cpp)
        add_dependencies(test_server ${rt_gui_EXPORTED_TARGETS})
        target_link_libraries(test_server PRIVATE server)

        add_executable(test_client include/rt_gui/rt_gui_client.h test/test_client.cpp)
        add_dependencies(test_client ${rt_gui_EXPORTED_TARGETS})
        target_link_libraries(test_client PRIVATE qt_utils ${catkin_LIBRARIES})

        add_executable(test_client_1 include/rt_gui/rt_gui_client.h test/test_client_1.cpp)
        add_dependencies(test_client_1 ${rt_gui_EXPORTED_TARGETS})
        target_link_libraries(test_client_1 PRIVATE qt_utils ${catkin_LIBRARIES})

        add_executable(test_console test/test_console.cpp)
        target_link_libraries(test_console PRIVATE console)
    endif()

endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


