add_library(qt_utils SHARED
    include/qt_utils/window.h
    include/qt_utils/slidersgroup.h
    src/qt_utils/window.cpp
    src/qt_utils/slidersgroup.cpp
    )

target_link_libraries(qt_utils PUBLIC
                      Qt5::Widgets
                      ${QWT_LIBRARIES})

install(TARGETS qt_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
