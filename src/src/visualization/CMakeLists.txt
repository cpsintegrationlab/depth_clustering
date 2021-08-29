# Find packages
find_package(OpenGL REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)

# Set Qt libraries                                                                             
set(
    QT_LIBRARIES
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
)

# Set include directories                                                                      
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    SYSTEM ${OpenGL_INCLUDE_DIRS}
    SYSTEM ${QGLVIEWER_INCLUDE_DIR}
)

# Add subdirectories             
add_subdirectory(drawables)
add_subdirectory(viewer)
add_subdirectory(utils)
add_subdirectory(widgets)