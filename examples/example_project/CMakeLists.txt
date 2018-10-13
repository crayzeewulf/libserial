CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(ExampleProject LANGUAGES C CXX)

SET(ExampleProject_VERSION_MAJOR 1)
SET(ExampleProject_VERSION_MINOR 0)

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
  -Wextra
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

SET(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
SET(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_INSTALL_MESSAGE ALWAYS)


ADD_EXECUTABLE(ExampleProject
  example_project.cpp
)

TARGET_LINK_LIBRARIES(ExampleProject
  libserial_static
)
