AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

noinst_PROGRAMS = unit_tests UnitTests

unit_tests_SOURCES = unit_tests.cpp

unit_tests_LDADD = \
	../src/libserial.la \
	-lboost_unit_test_framework

noinst_HEADERS = \
	SerialPortUnitTests.h \
	SerialStreamUnitTests.h \
	MultiThreadUnitTests.h \
	UnitTests.h

UnitTests_SOURCES = \
	SerialPortUnitTests.cpp \
	SerialStreamUnitTests.cpp \
	MultiThreadUnitTests.cpp \
	UnitTests.cpp

UnitTests_LDADD = \
	../src/libserial.la \
	-lgtest \
    -lgtest_main \
	-lpthread
