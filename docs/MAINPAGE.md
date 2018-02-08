\mainpage
Welcome to LibSerial
===

LibSerial provides a collection of C++ classes that allow object oriented
access to serial ports on POSIX systems.

The SerialPort class is available to simplified access to serial port
settings along with a set of convenient read/write methods.<br>
This class is useful for embedded systems where a complete C++ STL
may not be available.

The SerialStream class allows access to serial ports in the same manner as
standard C++ iostream objects.

Member functions are provided in both classes for setting serial port
parameters such as baud rate, character size, flow control, etc.

LibSerial exists to simplify serial port programming on POSIX systems.<br>
Here is short example using libserial:

\include main_page_example.cpp

In addition to the C++ programming languge, LibSerial releases after version
0.6.0 also provide bindings to several scripting languages such as Python,
Perl, PHP, Java, and Ruby. This provides developers a wide range languages to
select when writing applications that need access to serial ports on POSIX
compatible operating systems. LibSerial has received most extensive testing
under Linux operating system. 
