# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BUILD_SOURCE_DIRS "/Users/david/Documents/GitHub/mesh-Davidchen146;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release")
set(CPACK_CMAKE_GENERATOR "Ninja")
set(CPACK_COMPONENT_DEVEL_DESCRIPTION "Header Files for C and ISPC required to develop applications with Embree.")
set(CPACK_COMPONENT_DEVEL_DISPLAY_NAME "Development")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION "Tutorials demonstrating how to use Embree.")
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Examples")
set(CPACK_COMPONENT_LIB_DESCRIPTION "The Embree library including documentation.")
set(CPACK_COMPONENT_LIB_DISPLAY_NAME "Library")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/Users/david/Qt/Tools/CMake/CMake.app/Contents/share/cmake-3.27/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "mesh built using CMake")
set(CPACK_DMG_SLA_USE_RESOURCE_FILE_LICENSE "ON")
set(CPACK_GENERATOR "TBZ2;TGZ;TXZ;TZ")
set(CPACK_IGNORE_FILES "/CVS/;/\\.svn/;/\\.bzr/;/\\.hg/;/\\.git/;\\.swp\$;\\.#;/#")
set(CPACK_INNOSETUP_ARCHITECTURE "x64")
set(CPACK_INSTALLED_DIRECTORIES "/Users/david/Documents/GitHub/mesh-Davidchen146;/")
set(CPACK_INSTALL_CMAKE_PROJECTS "")
set(CPACK_INSTALL_PREFIX "/usr/local")
set(CPACK_MODULE_PATH "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/embree-src/common/cmake;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/libigl-src/cmake/recipes/external;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/libigl-src/cmake/;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/libigl-src/cmake/igl;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/libigl-src/cmake/find;/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/libigl-src/cmake/recipes/external;/Users/david/Documents/GitHub/mesh-Davidchen146/cmake")
set(CPACK_MONOLITHIC_INSTALL "1")
set(CPACK_NSIS_DISPLAY_NAME "Intel(R) Embree Ray Tracing Kernels 3.13.3")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "Intel(R) Embree Ray Tracing Kernels 3.13.3")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OBJDUMP_EXECUTABLE "/usr/bin/objdump")
set(CPACK_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX14.2.sdk")
set(CPACK_OUTPUT_CONFIG_FILE "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/CPackConfig.cmake")
set(CPACK_PACKAGE_CONTACT "embree_support@intel.com")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/Users/david/Qt/Tools/CMake/CMake.app/Contents/share/cmake-3.27/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Intel(R) Embree implements high performance ray tracing kernels including accelertion structure construction and traversal.")
set(CPACK_PACKAGE_FILE_NAME "Intel(R) Embree Ray Tracing Kernels-3.13.3-Source")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Intel(R) Embree Ray Tracing Kernels 3.13.3")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "Intel(R) Embree Ray Tracing Kernels 3.13.3")
set(CPACK_PACKAGE_NAME "Intel(R) Embree Ray Tracing Kernels")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "Intel Corporation")
set(CPACK_PACKAGE_VERSION "3.13.3")
set(CPACK_PACKAGE_VERSION_MAJOR "3")
set(CPACK_PACKAGE_VERSION_MINOR "13")
set(CPACK_PACKAGE_VERSION_PATCH "3")
set(CPACK_RESOURCE_FILE_LICENSE "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/embree-src/LICENSE.txt")
set(CPACK_RESOURCE_FILE_README "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/_deps/embree-build/README.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/Users/david/Qt/Tools/CMake/CMake.app/Contents/share/cmake-3.27/Templates/CPack.GenericWelcome.txt")
set(CPACK_RPM_PACKAGE_RELEASE "1")
set(CPACK_RPM_PACKAGE_SOURCES "ON")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;TXZ;TZ")
set(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\.svn/;/\\.bzr/;/\\.hg/;/\\.git/;\\.swp\$;\\.#;/#")
set(CPACK_SOURCE_INSTALLED_DIRECTORIES "/Users/david/Documents/GitHub/mesh-Davidchen146;/")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/CPackSourceConfig.cmake")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "Intel(R) Embree Ray Tracing Kernels-3.13.3-Source")
set(CPACK_SOURCE_RPM "OFF")
set(CPACK_SOURCE_TBZ2 "ON")
set(CPACK_SOURCE_TGZ "ON")
set(CPACK_SOURCE_TOPLEVEL_TAG "Darwin-Source")
set(CPACK_SOURCE_TXZ "ON")
set(CPACK_SOURCE_TZ "ON")
set(CPACK_SOURCE_ZIP "OFF")
set(CPACK_STRIP_FILES "")
set(CPACK_SYSTEM_NAME "Darwin")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "Darwin-Source")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/Users/david/Documents/GitHub/mesh-Davidchen146/build/build-mesh-Davidchen146-Qt_6_5_3_macos-Release/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
