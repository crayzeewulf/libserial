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
target_link_libraries(libserial_static Threads::Threads)

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

#
# The static library is always built as it is needed for unit tests but it is
# installed only if INSTALL_STATIC is set. 
#
if (INSTALL_STATIC)
    install(TARGETS libserial_static
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
#
# Use the same name (libserial) for both the static and shared libraries
#
set_target_properties(libserial_static PROPERTIES OUTPUT_NAME libserial)

#
# Build and install the shared object library only if INSTALL_SHARED is set.
#
if (INSTALL_SHARED)
    add_library(libserial_shared SHARED ${LIBSERIAL_SOURCES})
    set_target_properties(libserial_shared PROPERTIES PREFIX "")
    set_target_properties(libserial_shared PROPERTIES OUTPUT_NAME libserial)
    target_include_directories(libserial_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(libserial_shared Threads::Threads)
    #
    # Add version numbering to the shared library. Based on the recommendations in
    # the following book:
    #
    # "Professional CMake: A Practical Guide" --Craig Scott 
    # (https://crascit.com/professional-cmake/)
    #
    set_target_properties(libserial_shared PROPERTIES
        VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
    install(TARGETS libserial_shared
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
