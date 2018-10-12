AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

noinst_PROGRAMS = unit_tests UnitTests

unit_tests_SOURCES = unit_tests.cpp

unit_tests_LDADD = ../src/libserial.la -lboost_unit_test_framework

UnitTests_SOURCES = \
	SerialPortUnitTests.cpp \
	SerialStreamUnitTests.cpp \
	MultiThreadUnitTests.cpp \
	SerialPortUnitTests.cpp

UnitTests_LDADD = ../src/libserial.la /usr/lib/libgtest.a /usr/lib/libgtest_main.a -lpthread
