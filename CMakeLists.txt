#
# Minimum version is 3.5 (this supports Ubuntu 16.04 and later, for example)
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

#
# Project version numbering using semantic versioning. See:
# https://semver.org/
#
# These are used to set VERSION and SOVERSION properties of the LibSerial
# libarary. See:
# - https://cmake.org/cmake/help/latest/prop_tgt/SOVERSION.html
# - https://cmake.org/cmake/help/latest/prop_tgt/VERSION.html
#
PROJECT(LibSerial LANGUAGES C CXX VERSION 1.0.0)

option(LIBSERIAL_ENABLE_TESTING "Enables building unit tests" ON)
option(LIBSERIAL_BUILD_EXAMPLES "Enables building example programs" ON)
option(LIBSERIAL_PYTHON_ENABLE "Enables building the library with Python SIP bindings" ON)
option(LIBSERIAL_BUILD_DOCS "Build the Doxygen docs" ON)

#
# Project specific options and variables
#
OPTION(INSTALL_STATIC "Install static library." ON)
OPTION(INSTALL_SHARED "Install shared object library." ON)

#
# LibSerial requies a C++ compiler that supports at least C++14 standard
#
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_STANDARD_REQUIRES ON)

INCLUDE(ExternalProject)

if (LIBSERIAL_ENABLE_TESTING)
  ENABLE_TESTING()
endif()
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Create compile_commands.json file so that it may be used by various
# editors/plugins/IDEs that support it.
#
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

#
# Use GNU standard installation directories. CMake will use /usr/local
# as the default install directory. Users may override this by setting
# CMAKE_INSTALL_PREFIX. For example:
#
# cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr ..
#
INCLUDE(GNUInstallDirs)

#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to FIND_PACKAGE(Threads).
#
SET(THREADS_HAVE_PTHREAD_ARG 1)
if (LIBSERIAL_BUILD_DOCS)
  FIND_PACKAGE(Doxygen REQUIRED)
endif()
if (LIBSERIAL_ENABLE_TESTING)
  FIND_PACKAGE(Boost COMPONENTS unit_test_framework REQUIRED)
endif()
if (LIBSERIAL_PYTHON_ENABLE)
  FIND_PACKAGE(PythonLibs REQUIRED)
endif()
#FIND_PACKAGE(SIP REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

#
# Use -DCMAKE_BUILD_TYPE=Release or -DCMAKE_BUILD_TYPE=Debug to let CMake
# decide whether to use debug or optimization flags. We should not hard-code
# them here. Similarly, let CMake handle flags needed for shared object files
# (such as -fPIC). Additionally, "-pthread" flag will also be handled by CMake
# via the use of CMAKE_THREAD_LIBS_INIT (cmake < 3.1) or Threads::Threads.
#
ADD_DEFINITIONS(
  -Wall
  -Wcast-align
  -Wchar-subscripts
  -Wchkp
  -Wdouble-promotion
  -Wextra
  -Wfatal-errors
  -Wformat
  -Wformat-security
  -Wlogical-op
  -Wno-format-extra-args
  -Wno-long-long
  -Wno-parentheses
  -Wno-psabi
  -Wno-variadic-macros
  -Woverlength-strings
  -Wpacked
  -Wpointer-arith
  -Wunused-local-typedefs
  -Wwrite-strings

  -fstrict-aliasing
  -fno-check-new
  -fno-common
  -fvisibility=default
  -pedantic
  )

if (LIBSERIAL_ENABLE_TESTING)
  SET(GTEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtest")

  EXTERNALPROJECT_ADD(GTestExternal
    PREFIX             "${GTEST_PREFIX}"
    URL https://github.com/google/googletest/archive/release-1.8.1.tar.gz
    URL_HASH SHA1=152b849610d91a9dfa1401293f43230c2e0c33f8
    INSTALL_COMMAND ""
    )

  SET(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
  SET(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")

  SET(GTEST_LOCATION "${GTEST_PREFIX}/src/GTestExternal-build/googlemock/gtest/")
  SET(GTEST_LIBRARY  "${GTEST_LOCATION}/${LIBPREFIX}gtest${LIBSUFFIX}")
  SET(GTEST_MAINLIB  "${GTEST_LOCATION}/${LIBPREFIX}gtest_main${LIBSUFFIX}")

  ADD_LIBRARY(GTest IMPORTED STATIC GLOBAL)
  SET_TARGET_PROPERTIES(GTest
    PROPERTIES
    IMPORTED_LOCATION "${GTEST_LIBRARY}"
    IMPORTED_LINK_INTERFACE_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}"
    )

  ADD_LIBRARY(GTestMain IMPORTED STATIC GLOBAL)
  SET_TARGET_PROPERTIES(GTestMain
    PROPERTIES
    IMPORTED_LOCATION "${GTEST_MAINLIB}"
    IMPORTED_LINK_INTERFACE_LIBRARIES "${GTEST_LIBRARY};${CMAKE_THREAD_LIBS_INIT}"
    )

  ADD_DEPENDENCIES(GTest GTestExternal)
  ADD_DEPENDENCIES(GTestMain GTestExternal)

  EXTERNALPROJECT_GET_PROPERTY(GTestExternal source_dir)

  INCLUDE_DIRECTORIES(
    BEFORE ${GTEST_PREFIX}/src/GTestExternal/googletest/include
    ${Boost_INCLUDE_DIRS}
    )
endif()

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_INSTALL_MESSAGE ALWAYS)

if (LIBSERIAL_BUILD_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
endif()
if (LIBSERIAL_PYTHON_ENABLE)
  ADD_SUBDIRECTORY(sip)
endif()
ADD_SUBDIRECTORY(src)
if (LIBSERIAL_ENABLE_TESTING)
  ADD_SUBDIRECTORY(test)
endif()

if (LIBSERIAL_BUILD_DOCS)
  CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf.in @ONLY
    )

  ADD_CUSTOM_TARGET(docs ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif()
