Description
===========

LibSerial provides a collection of C++ classes that allow one to access serial
ports on POSIX systems like standard C++ iostream objects. The SerialStream
class represents the primary class that is expected to be used by developers to
access serial ports as iostream objects. Additionally a SerialPort class is
available to provide access to serial ports without using the standard C++
iostream interface. This is useful for embedded systems where a complete C++
STL may not be available. Member functions are provided for setting various
parameters of the serial ports such as the baud rate, character size, flow
control and others. The idea is to simplify serial port programming on POSIX
systems. For example, using libserial, you can do the following:

.. code-block:: c++

   #include <SerialStream.h>
   //
   // Open the serial port. 
   //
   SerialStream serial_port( "/dev/ttyS0" ) ;
   //
   // Set the baud rate of the serial port.
   //
   serial_port.SetBaudRate( SerialStreamBuf::BAUD_9600 ) ;
   //
   // Read a character from the serial port. 
   //
   char next_char ; 
   serial_port >> next_char ; 
   //
   // Write the character back to the serial port. 
   //
   serial_port << next_char ;

In addition to the C++ programming languge, LibSerial releases after version
0.6.0 also provide bindings to several scripting languages such as Python,
Perl, PHP, Java, and Ruby. This provides developers a wide range languages to
select when writing applications that need access to serial ports on POSIX
compatible operating systems. LibSerial has received most extensive testing
under Linux operating system. 
