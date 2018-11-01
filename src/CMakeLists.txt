set(LIBSERIAL_SOURCES
    SerialPort.cpp
    SerialStream.cpp
    SerialStreamBuf.cpp)

add_library(libserial_static STATIC ${LIBSERIAL_SOURCES})

#
# We already have "lib" prefix in the target name. Prevent CMake from adding
# another "lib" prefix.
#
set_target_properties(libserial_static PROPERTIES PREFIX "")

target_include_directories(libserial_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (CMAKE_VERSION VERSION_LESS "3.1")
    TARGET_LINK_LIBRARIES(libserial_static ${CMAKE_THREAD_LIBS_INIT})
else()
    TARGET_LINK_LIBRARIES(libserial_static Threads::Threads)
endif()

#
# Install all our headers under the libserial subfolder of the include
# directory so that our headers do not conflict with similarly named headers
# from other libraries. Clients can either include our headers with libserial
# folder prefixed (for example, "#include <libserial/SerialPort.h>") or put the
# libserial folder in their include path (for example,
# "-I/usr/include/libserial").
#
install(DIRECTORY libserial
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS libserial_static
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

set_target_properties(libserial_static PROPERTIES OUTPUT_NAME libserial)
