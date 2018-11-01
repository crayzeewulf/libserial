Description
===========

LibSerial was created to simplify serial port programming on POSIX
systems through a collection of object oriented C++ classes.

The `SerialPort` class allows simplified access to serial port
settings and usage through a convenient set of methods.
This class is useful for embedded systems where a complete C++ STL
may not be available.

The `SerialStream` class allows access to serial ports in the same manner as
standard C++ iostream objects.

Methods are provided for setting serial port parameters
such as baud rate, character size, flow control, etc.

Here is short example using libserial:

.. code-block:: c++

   #include <libserial/SerialPort.h>
   #include <libserial/SerialStream.h>

   using namespace LibSerial;

   int main()
   {
      // Instantiate a Serial Port and a Serial Stream object.
      SerialPort serial_port;
      SerialStream serial_stream;

      // Open the hardware serial ports.
      serial_port.Open( "/dev/ttyUSB0" );
      serial_stream.Open( "/dev/ttyUSB1" );

      // Set the baud rates.
      serial_port.SetBaudRate( BaudRate::BAUD_115200 );
      serial_stream.SetBaudRate( BaudRate::BAUD_115200 );

      char write_byte_1 = 'a';
      char write_byte_2 = 'b';

      char read_byte_1 = 'A';
      char read_byte_2 = 'B';

      // Write a character.
      serial_port.Write(&write_byte_1, 1);
      serial_stream << write_byte_2;

      // Read a character.
      serial_port.Read(read_byte_1, 1);
      serial_stream >> read_byte_2;

      std::cout << "serial_port read:   " << read_byte_1 << std::endl;
      std::cout << "serial_stream read: " << read_byte_2 << std::endl;

      // Close the Serial Port and Serial Stream.
      serial_port.Close();
      serial_stream.Close();
   }

In addition to the C++ programming languge, LibSerial releases after version
0.6.0 also provide bindings to several scripting languages such as Python,
Perl, PHP, Java, and Ruby. This provides developers a wide range languages to
select when writing applications that need access to serial ports on POSIX
compatible operating systems. LibSerial has received the most extensive testing
on (Debian) Linux operating systems.
