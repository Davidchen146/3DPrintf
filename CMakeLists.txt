cmake_minimum_required(VERSION 3.16)

# Sets project name
project(mesh LANGUAGES CXX)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

igl_include(embree)
igl_include(core)

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
    main.cpp
    mesh.cpp

    mesh.h
    util/tiny_obj_loader.h
)

# Specifies libraries to be linked (Qt components, glew, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
    igl::embree
    igl::core
)

# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE
    Eigen
)

# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()


# cmake_minimum_required(VERSION 3.16)
# project(example)

# list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# # Libigl
# include(libigl)

# # Enable the target igl::glfw
# igl_include(glfw)
# # Other modules you could enable
# igl_include(embree)
# #igl_include(imgui)
# igl_include(opengl)
# #igl_include(stb)
# #igl_include(predicates)
# #igl_include(xml)
# #igl_include(copyleft cgal)
# #igl_include(copyleft comiso)
# #igl_include(copyleft core)
# #igl_include(copyleft cork)
# #igl_include(copyleft tetgen)
# #igl_include(restricted matlab)
# #igl_include(restricted mosek)
# #igl_include(restricted triangle)

# # Add your project files
# file(GLOB SRC_FILES *.cpp)
# add_executable(${PROJECT_NAME} ${SRC_FILES})

# # Link igl (and the glfw module) to your project
# target_link_libraries(${PROJECT_NAME} PUBLIC
#   igl::glfw
#   ## Other modules you could link to
#   igl::embree
#   # igl::imgui
#   igl::opengl
#   # igl::stb
#   # igl::predicates
#   # igl::xml
#   # igl_copyleft::cgal
#   # igl_copyleft::comiso
#   # igl_copyleft::core
#   # igl_copyleft::cork
#   # igl_copyleft::tetgen
#   # igl_restricted::matlab
#   # igl_restricted::mosek
#   # igl_restricted::triangle
#   )
