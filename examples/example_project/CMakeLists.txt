cmake_minimum_required(VERSION 3.8)

project(ExampleProject LANGUAGES C CXX)

set(ExampleProject_VERSION_MAJOR 1)
set(ExampleProject_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#
# Use GNU standard installation directories. CMake will use /usr/local
# as the default install directory. Users may override this by setting
# CMAKE_INSTALL_PREFIX. For example:
#
# cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr ..
#

include(GNUInstallDirs)

#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to find_package(Threads).
#
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig)
#pkg_check_modules(SERIAL libserial)

#
# Use -DCMAKE_BUILD_TYPE=Release or -DCMAKE_BUILD_TYPE=Debug to let CMake
# decide whether to use debug or optimization flags. We should not hard-code
# them here. Similarly, let CMake handle flags needed for shared object files
# (such as -fPIC). Additionally, "-pthread" flag will also be handled by CMake
# via the use of CMAKE_THREAD_LIBS_INIT (cmake < 3.1) or Threads::Threads.
#
add_definitions(
  -Wall
  -Wextra
  )

set(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
set(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(ExampleProject
  example_project.cpp
)

target_link_libraries(ExampleProject
  libserial_static.a
)
