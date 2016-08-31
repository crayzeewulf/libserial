CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(LibSerial)

INCLUDE(ExternalProject)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(Threads)

ADD_DEFINITIONS(
  -O3
  -DAPL=0
  -DIBM=0
  -DLIN=1
  -DNDEBUG
  -DXPLM200
  -DXPLM210

  -Wall
  -Wcast-align
  -Wchar-subscripts
  -Wdouble-promotion
  -Wextra
  -Wformat-security
  -Wlogical-op
  -Wno-long-long  
  -Wno-psabi
  -Wno-variadic-macros
  #-Wnon-virtual-dtor
  -Wpointer-arith
  #-Wundef
  -Wuninitialized
  -Wunused-local-typedefs
  -Wwrite-strings

  -fstrict-aliasing
  -fno-check-new
  -fno-common
  -fvisibility=hidden
  -fPIC
  -c
  -g
  -pedantic
  -pthread
  )

# Uncomment the following lines to compile for 32 bit architectures
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -m32")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -m32")

# Uncomment the following lines to compile for 64 bit architectures
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")

SET(GTEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtest")

EXTERNALPROJECT_ADD(GTestExternal
  PREFIX             "${GTEST_PREFIX}"
  URL https://github.com/google/googletest/archive/release-1.7.0.tar.gz
  URL_HASH SHA1=d7aa4b8536f3a007b480cf016be8a4333dbf4768
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

SET(GTEST_LOCATION "${GTEST_PREFIX}/src/GTestExternal-build")
SET(GTEST_LIBRARY  "${GTEST_LOCATION}/${LIBPREFIX}gtest${LIBSUFFIX}")
SET(GTEST_MAINLIB  "${GTEST_LOCATION}/${LIBPREFIX}gtest_main${LIBSUFFIX}")

ADD_LIBRARY(GTest IMPORTED STATIC GLOBAL)
SET_TARGET_PROPERTIES(GTest PROPERTIES
    IMPORTED_LOCATION
    "${GTEST_LIBRARY}"
    IMPORTED_LINK_INTERFACE_LIBRARIES
    "${CMAKE_THREAD_LIBS_INIT}"
    )

ADD_LIBRARY(GTestMain IMPORTED STATIC GLOBAL)
SET_TARGET_PROPERTIES(GTestMain PROPERTIES
    IMPORTED_LOCATION
    "${GTEST_MAINLIB}"
    IMPORTED_LINK_INTERFACE_LIBRARIES
    "${GTEST_LIBRARY};${CMAKE_THREAD_LIBS_INIT}"
    )

ADD_DEPENDENCIES(GTest GTestExternal)
ADD_DEPENDENCIES(GTestMain GTestExternal)

EXTERNALPROJECT_GET_PROPERTY(GTestExternal source_dir)

INCLUDE_DIRECTORIES(
  src
  )

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

ADD_SUBDIRECTORY(examples)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)
