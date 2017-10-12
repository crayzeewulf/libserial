AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

lib_LTLIBRARIES = libserial.la

include_HEADERS = \
	SerialPort.h \
	SerialStream.h \
	SerialStreamBuf.h

libserial_la_SOURCES = \
	SerialPort.cpp \
	SerialPort.h \
	SerialStream.cc \
	SerialStream.h \
	SerialStreamBuf.cc \
	SerialStreamBuf.h \
	PosixSignalDispatcher.cpp

noinst_HEADERS = \
	PosixSignalDispatcher.h \
	PosixSignalHandler.h
