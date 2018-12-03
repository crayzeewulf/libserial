AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

SUBDIRS = libserial

lib_LTLIBRARIES = libserial.la

libserial_la_SOURCES = \
	SerialPort.cpp \
	SerialStream.cpp \
	SerialStreamBuf.cpp

libserialincludedir = @includedir@/libserial
libserialinclude_HEADERS = \
	libserial/SerialPort.h \
	libserial/SerialPortConstants.h \
	libserial/SerialStream.h \
	libserial/SerialStreamBuf.h

libserial_la_LDFLAGS = -version-info 1:0:0
