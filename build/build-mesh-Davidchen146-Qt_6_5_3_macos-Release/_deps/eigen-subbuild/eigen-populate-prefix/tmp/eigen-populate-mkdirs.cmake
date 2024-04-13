# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-src"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-build"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/tmp"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/src"
  "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
