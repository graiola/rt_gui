cmake_minimum_required(VERSION 3.10)
project(rt_gui_msgs)

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

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})

    find_package(message_generation REQUIRED)
    find_package(std_msgs REQUIRED)

    add_service_files(
      FILES
      Bool.srv
      Double.srv
      Int.srv
      List.srv
      Void.srv
      Text.srv
    )

    generate_messages()

    catkin_package()

elseif(${ament_cmake_FOUND})

    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/Bool.srv"
        "srv/Double.srv"
        "srv/Int.srv"
        "srv/List.srv"
        "srv/Void.srv"
        "srv/Text.srv"
        DEPENDENCIES std_msgs std_srvs
      )

    ament_export_dependencies(rosidl_default_runtime)

    ament_package()

endif()
