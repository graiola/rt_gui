cmake_minimum_required(VERSION 3.1.0)
project(rt_gui)

add_compile_options(-std=c++17)

option(QT_VERSION_GREATER_THAN_5_5 "Set to true if Qt5 > 5.5 is installed" FALSE)

set(CMAKE_AUTOMOC ON)

if(${QT_VERSION_GREATER_THAN_5_5})
    set(MINIMUM_QT_VERSION 5.9)
else()
    set(MINIMUM_QT_VERSION 5.5)
endif()

## Get custom path to QT from env
message(STATUS "Searching QT in '${QT_PATH}'")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Get CMAKE_PREFIX_PATH from environment, and append QT_PATH
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
string(REPLACE ":" ";" CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})

find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qwt ${MINIMUM_QT_VERSION} REQUIRED)
find_package(catkin REQUIRED roscpp rospy std_msgs message_generation)

add_service_files(
  FILES
  Bool.srv
  Double.srv
  Int.srv
  List.srv
  Void.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qt_utils server
  CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIRS}
)

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
add_subdirectory(bindings/python/)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/support
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS server_node qt_utils server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

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

    add_executable(test_console test/test_console.cpp)
    target_link_libraries(test_console PRIVATE console)
endif()
