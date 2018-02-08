AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

noinst_PROGRAMS = \
	main_page_example \
	serial_port_read \
	serial_port_read_write \
	serial_port_write \
	serial_stream_read \
	serial_stream_read_write \
	serial_stream_write 

main_page_example_SOURCES = main_page_example.cpp
main_page_example_LDADD = ../src/libserial.la

serial_port_read_SOURCES = serial_port_read.cpp
serial_port_read_LDADD = ../src/libserial.la

serial_port_read_write_SOURCES = serial_port_read_write.cpp
serial_port_read_write_LDADD = ../src/libserial.la

serial_port_write_SOURCES = serial_port_write.cpp
serial_port_write_LDADD = ../src/libserial.la

serial_stream_read_SOURCES = serial_stream_read.cpp
serial_stream_read_LDADD = ../src/libserial.la

serial_stream_read_write_SOURCES = serial_stream_read_write.cpp
serial_stream_read_write_LDADD = ../src/libserial.la

serial_stream_write_SOURCES = serial_stream_write.cpp
serial_stream_write_LDADD = ../src/libserial.la
