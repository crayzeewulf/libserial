SET(LIBSERIAL_SOURCES
    SerialPort.cpp
    SerialStream.cpp
    SerialStreamBuf.cpp)

ADD_LIBRARY(libserial_static STATIC ${LIBSERIAL_SOURCES})

#
# We already have "lib" prefix in the target name. Prevent CMake from adding
# another "lib" prefix.
#
SET_TARGET_PROPERTIES(libserial_static PROPERTIES PREFIX "")

TARGET_INCLUDE_DIRECTORIES(libserial_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

IF (CMAKE_VERSION VERSION_LESS "3.1")
    TARGET_LINK_LIBRARIES(libserial_static ${CMAKE_THREAD_LIBS_INIT})
ELSE()
    TARGET_LINK_LIBRARIES(libserial_static Threads::Threads)
ENDIF()

#
# Install all our headers under the libserial subfolder of the include
# directory so that our headers do not conflict with similarly named headers
# from other libraries. Clients can either include our headers with libserial
# folder prefixed (for example, "#include <libserial/SerialPort.h>") or put the
# libserial folder in their include path (for example,
# "-I/usr/include/libserial").
#
INSTALL(DIRECTORY libserial
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

INSTALL(TARGETS libserial_static
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
