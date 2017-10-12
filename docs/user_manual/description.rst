Description
===========

LibSerial provides a collection of C++ classes that allow one to access serial
ports on POSIX systems like standard C++ iostream objects.

The SerialStream class represents the primary class that is expected to be used
to access serial ports as iostream objects. 

The SerialPort class is available to provide access to serial ports without
using the standard C++ iostream interface. This is useful for embedded systems
where a complete C++ STL may not be available.

Member functions are provided in both classes for setting various parameters
of the serial ports such as baud rate, character size, flow control and others.
The idea is to simplify serial port programming on POSIX systems.
For example, using libserial, you can do the following:

.. code-block:: c++

   #include <SerialPort.h>
   #include <SerialStream.h>

   using namespace Libserial

   // Open a Serial Port and a Serial Stream.
   SerialPort serial_port( "/dev/ttyUSB0" );
   SerialStream serial_stream( "/dev/ttyUSB1" );

   // Set the baud rate.
   serial_port.SetBaudRate( BAUD_115200 );
   serial_stream.SetBaudRate( BAUD_115200 );

   // Read a character.
   char next_char;
   serial_port.Read(next_char, 25);
   serial_stream >> next_char;

   // Write a character.
   serial_port.Write(next_char);
   serial_stream << next_char;

   // Close the Serial Port and Serial Stream
   serial_port.Close();
   serial_stream.Close();

In addition to the C++ programming languge, LibSerial releases after version
0.6.0 also provide bindings to several scripting languages such as Python,
Perl, PHP, Java, and Ruby. This provides developers a wide range languages to
select when writing applications that need access to serial ports on POSIX
compatible operating systems. LibSerial has received most extensive testing
under Linux operating system. 
