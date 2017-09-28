AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

noinst_PROGRAMS = unit_tests LibSerialTests

unit_tests_SOURCES = unit_tests.cpp
unit_tests_LDADD = ../src/libserial.la -lboost_unit_test_framework

LibSerialTests_SOURCES = LibSerialTests.cpp
LibSerialTests_LDADD = ../src/libserial.la /usr/lib/libgtest.a /usr/lib/libgtest_main.a -lpthread