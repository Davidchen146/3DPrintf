cmake_minimum_required(VERSION 3.16)

# Sets project name
project(3DPrintf LANGUAGES CXX)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

igl_include(embree)
igl_include(core)
igl_include(glfw)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Sets C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specifies required Qt components
find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Gui)
find_package(Qt6 REQUIRED COMPONENTS Xml)

# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mesh.cpp
    src/meshoperations.cpp
    src/mesh.h
    src/meshoperations.h
    util/tiny_obj_loader.h
    src/meshoperations.h
    src/meshoperations.cpp
    src/oversegmentation.cpp
    src/meshop_util.cpp
)

# Specifies libraries to be linked (Qt components, glew, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
    igl::embree
    igl::core
    igl::glfw
)

# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE
    Eigen
)

# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
    find_package(Qt6 REQUIRED COMPONENTS Core)
    qt_disable_unicode_defines(${PROJECT_NAME})
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
