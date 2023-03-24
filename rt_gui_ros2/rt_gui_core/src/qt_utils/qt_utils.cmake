add_library(qt_utils SHARED
    include/rt_gui/qt_utils/window.h
    include/rt_gui/qt_utils/double_slider.h
    include/rt_gui/qt_utils/int_slider.h
    include/rt_gui/qt_utils/radio_button.h
    include/rt_gui/qt_utils/combo_box.h
    include/rt_gui/qt_utils/button.h
    include/rt_gui/qt_utils/text.h
    include/rt_gui/qt_utils/label.h
    src/qt_utils/window.cpp
    src/qt_utils/double_slider.cpp
    src/qt_utils/int_slider.cpp
    src/qt_utils/radio_button.cpp
    src/qt_utils/combo_box.cpp
    src/qt_utils/button.cpp
    src/qt_utils/text.cpp
    src/qt_utils/label.cpp
    )

target_link_libraries(qt_utils PUBLIC
                      Qt5::Widgets
                      ${QWT_LIBRARIES})

install(TARGETS qt_utils
  ARCHIVE DESTINATION ${LIB_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
  RUNTIME DESTINATION ${BIN_DESTINATION}
)
