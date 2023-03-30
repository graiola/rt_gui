cmake_minimum_required(VERSION 3.5)
project(rt_gui_core)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(Qt5Core ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qwt ${MINIMUM_QT_VERSION} REQUIRED)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package(INCLUDE_DIRS include LIBRARIES qt_utils console DEPENDS pthread readline Qwt Qt5Core Qt5Widgets)
elseif(${ament_cmake_FOUND})
    ament_export_include_directories(include)
    ament_export_libraries(qt_utils console)
    ament_package()
endif()

include_directories(
    include
    ${QWT_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

## Compile qt_utils lib
add_library(qt_utils SHARED
    include/${PROJECT_NAME}/qt_utils/window.h
    include/${PROJECT_NAME}/qt_utils/double_slider.h
    include/${PROJECT_NAME}/qt_utils/int_slider.h
    include/${PROJECT_NAME}/qt_utils/radio_button.h
    include/${PROJECT_NAME}/qt_utils/combo_box.h
    include/${PROJECT_NAME}/qt_utils/button.h
    include/${PROJECT_NAME}/qt_utils/text.h
    include/${PROJECT_NAME}/qt_utils/label.h
    src/qt_utils/window.cpp
    src/qt_utils/double_slider.cpp
    src/qt_utils/int_slider.cpp
    src/qt_utils/radio_button.cpp
    src/qt_utils/combo_box.cpp
    src/qt_utils/button.cpp
    src/qt_utils/text.cpp
    src/qt_utils/label.cpp
    )
target_link_libraries(qt_utils PUBLIC Qt5::Core Qt5::Widgets ${QWT_LIBRARIES})

## Compile console lib
add_library(console src/console/console_core.cpp src/console/console.cpp)
target_link_libraries(console PUBLIC readline pthread)

## Compile test_qt
add_executable(test_qt test/test_qt.cpp)
target_link_libraries(test_qt PUBLIC qt_utils)

## Compile test_console
add_executable(test_console test/test_console.cpp)
target_link_libraries(test_console PUBLIC console)

## Install
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(
  TARGETS test_console test_qt console qt_utils
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
