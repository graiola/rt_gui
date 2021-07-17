add_library(qt_utils SHARED
    include/qt_utils/window.h
    include/qt_utils/slider.h
    include/qt_utils/radio_button.h
    include/qt_utils/combo_box.h
    src/qt_utils/window.cpp
    src/qt_utils/slider.cpp
    src/qt_utils/radio_button.cpp
    src/qt_utils/combo_box.cpp
    )

target_link_libraries(qt_utils PUBLIC
                      Qt5::Widgets
                      ${QWT_LIBRARIES})

install(TARGETS qt_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
