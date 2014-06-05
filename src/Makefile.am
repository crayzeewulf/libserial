AM_CXXFLAGS = -Weffc++

#noinst_PROGRAMS = unit_tests

lib_LTLIBRARIES = libserial.la

include_HEADERS = SerialStreamBuf.h SerialStream.h SerialPort.h

libserial_la_SOURCES = SerialStreamBuf.cc SerialStreamBuf.h SerialStream.cc \
		SerialStream.h SerialPort.cpp SerialPort.h PosixSignalDispatcher.cpp

#unit_tests_SOURCES = unit_tests.cpp
#unit_tests_LDADD = libserial.la -lboost_unit_test_framework-mt

noinst_HEADERS = PosixSignalDispatcher.h PosixSignalHandler.h
