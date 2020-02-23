# Add executables
add_executable(qt_gui_app main.cpp)

# Set library link targets
target_link_libraries(qt_gui_app qt_widgets)

# Add subdirectories             
add_subdirectory(drawables)
add_subdirectory(viewer)
add_subdirectory(utils)
add_subdirectory(widgets)
