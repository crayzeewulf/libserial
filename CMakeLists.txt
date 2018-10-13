CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(LibSerial LANGUAGES C CXX)

SET(LibSerial_VERSION_MAJOR 1)
SET(LibSerial_VERSION_MINOR 0)

INCLUDE(ExternalProject)

ENABLE_TESTING()

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
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
FIND_PACKAGE(Doxygen REQUIRED)
FIND_PACKAGE(Boost COMPONENTS unit_test_framework REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)
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

#
# We require support for C++11 features in the compiler. This can be
# achieved using CMAKE_CXX_STANDARD in CMake version 3.1 or later. For
# older versions we have to add the compiler flags manually.
#
macro(USE_CXX11)
    IF (CMAKE_VERSION VERSION_LESS "3.1")
        IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
        ENDIF ()
    ELSE ()
        SET(CMAKE_CXX_STANDARD 14)
        SET(CMAKE_STANDARD_REQUIRES ON)
    ENDIF ()
endmacro(USE_CXX11)
USE_CXX11()

SET(GTEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtest")

EXTERNALPROJECT_ADD(GTestExternal
  PREFIX             "${GTEST_PREFIX}"
  URL https://github.com/google/googletest/archive/release-1.8.0.tar.gz
  URL_HASH SHA1=e7e646a6204638fe8e87e165292b8dd9cd4c36ed
  INSTALL_COMMAND ""

  #@TODO - Pull from Github rather than prebuilt zip file.
  #GIT_REPOSITORY     https://github.com/google/googletest.git
  #GIT_TAG            master
  #SOURCE_DIR        "${GTEST_PREFIX}/src"
  #BINARY_DIR        "${GTEST_PREFIX}/build"
  #CONFIGURE_COMMAND ""
  #BUILD_COMMAND     ""
  #INSTALL_COMMAND   ""
  #TEST_COMMAND      ""
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

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_INSTALL_MESSAGE ALWAYS)

#ADD_SUBDIRECTORY(docs)
ADD_SUBDIRECTORY(examples)
ADD_SUBDIRECTORY(sip)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in
  ${CMAKE_CURRENT_BINARY_DIR}/doxygen.conf.in @ONLY
  )

ADD_CUSTOM_TARGET(docs ALL
  ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.conf.in
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating API documentation with Doxygen" VERBATIM
)

