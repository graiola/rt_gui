cmake_minimum_required(VERSION 3.1.0)
project(rt_gui)

add_compile_options(-std=c++14)

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

find_package(Qt5UiTools ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qwt ${MINIMUM_QT_VERSION} REQUIRED)
find_package(catkin REQUIRED roscpp rospy std_msgs message_generation)

add_service_files(
  FILES
  addSlider.srv
  updateClient.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QWT_INCLUDE_DIRS}
)

include(src/qt_utils/qt_utils.cmake)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Compile test
add_executable(test_qt test/test_qt.cpp)
target_link_libraries(test_qt PRIVATE qt_utils)
set_target_properties(test_qt PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(test_server include/rt_gui/rt_gui_server.h test/test_server.cpp)
target_link_libraries(test_server PRIVATE qt_utils ${catkin_LIBRARIES})
set_target_properties(test_server PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(test_client include/rt_gui/rt_gui_client.h test/test_client.cpp)
target_link_libraries(test_client PRIVATE qt_utils ${catkin_LIBRARIES})
set_target_properties(test_client PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS test_qt test_server test_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
