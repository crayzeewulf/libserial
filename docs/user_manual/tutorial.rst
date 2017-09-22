Tutorial
========

Opening a Serial Port I/O Stream
--------------------------------

A serial port I/O stream, SerialStream, instance can be created and opened by
providing the name of the serial port device to the constructor:

.. code-block:: c++ 

   #include <SerialStream.h>

   using namespace LibSerial ;

   // Create and open the serial port for communication. 
   SerialStream my_serial_stream( "/dev/ttyS0" ) ;

In certain applications, the name of the serial port device may not be known
when the SerialStream instance is created. In such cases, the same effect as
above can be achieved as follows:

.. code-block:: c++

   #include <SerialStream.h>

   using namespace LibSerial ;

   // Create a SerialStream instance.
   SerialStream my_serial_stream ;

   // Open the serial port for communication.
   my_serial_stream.Open( "/dev/ttyS0" ) ;

Setting the Baud Rate
---------------------

The baud rate for the SerialStream can be set using the
SerialStream::SetBaudRate() member function.

.. code-block:: c++

   // Set the desired baud rate using a SetBaudRate() method call.
   // Available baud rate values are defined in SerialStreamConstants.h. 
   my_serial_stream.SetBaudRate( LibSerial::BAUD_115200 );

Setting the Character Size
--------------------------

.. code-block:: c++

   // Set the desired character size using a SetCharacterSize() method call.
   // Available character size values are defined in SerialStreamConstants.h. 
   my_serial_port.SetCharacterSize( LibSerial::CHAR_SIZE_8 );

Setting the Flow-Control Type
-----------------------------

.. code-block:: c++ 

   // Set the desired flow control type using a SetClowControl() method call.
   // Available flow control types are defined in SerialStreamConstants.h. 
   my_serial_port.SetFlowControl( LibSerial::FLOW_CONTROL_HARD );


Setting the Parity Type
-----------------------

.. code-block:: c++

   // Set the desired parity type using a SetParity() method call.
   // Available parity types are defined in SerialStreamConstants.h. 
   my_serial_port.SetParity( LibSerial::PARITY_ODD );


Setting the Number of Stop Bits
-------------------------------

.. code-block:: c++

   // Set the number of stop bits using a SetNumberOfStopBits() method call.
   // Available stop bit values are defined in SerialStreamConstants.h. 
   my_serial_port.SetNumberOfStopBits(1) ;


Reading Characters
------------------

Characters can be read from the serial port using standard iostream ">>"
operator. For example:

.. code-block:: c++ 


   // Read one character from the serial port. 
   char next_char;
   my_serial_stream >> next_char;

   // You can also read other types of values from the serial port in a similar fashion. 
   int data_size;
   my_serial_stream >> data_size;

All other methods of standard C++ iostream objects are available too. For
example, one can read characters from the serial port using the get() method:

.. code-block:: c++

   // Read one byte from the serial port. 
   char next_byte;
   my_serial_stream.get( next_byte );

Writing Characters
------------------

.. code-block:: c++ 

   // Write a single character to the serial port. 
   my_serial_stream << 'U' ;

   // You can write a whole string. 
   my_serial_stream << "Hello, Serial Port." << std::endl ;

   // In fact, you can pretty much write any type of object that 
   // is supported by a "<<" operator. 
   double radius = 2.0 ;
   double area = M_PI * 2.0 * 2.0 ;
   my_serial_stream << area << std::endl ;

Reading Blocks of Data
----------------------

.. code-block:: c++ 

   // Read a whole array of data from the serial port. 
   const int BUFFER_SIZE = 256;
   char input_buffer[BUFFER_SIZE];

   my_serial_stream.read( input_buffer, 
                          BUFFER_SIZE );

Writing Blocks of Data
----------------------

.. code-block:: c++

   // Write an array of data from the serial port. 
   const int BUFFER_SIZE = 256;
   char output_buffer[BUFFER_SIZE];

   for(int i=0; i<BUFFER_SIZE; ++i) 
   {
       output_buffer[i] = i;
   }

   my_serial_stream.write( output_buffer, 
                           BUFFER_SIZE );

Closing the Serial Port
-----------------------

.. code-block:: c++ 

   my_serial_port.Close() ;

Complete Example Programs:
Reading from a SerialPort:
-------------------------

.. code-block:: c++

   #include <SerialPort.h>
   #include <iostream>
   #include <unistd.h>
   #include <cstdlib>

   using namespace LibSerial;

   int main()
   {
       // Instantiate a SerialPort object and open the serial port.
       SerialPort serial_port;

       serial_port.Open("/dev/ttyUSB0");

       // Check that the serial stream has opened correctly.
       if (!serial_port.IsOpen()) 
       {
           std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                     << "Error: Could not open serial port." 
                     << std::endl;
           exit(1);
       }

       // Set the baud rate of the serial port.
       serial_port.SetBaudRate( BaudRate::BAUD_115200 );

       // Set the number of data bits.
       serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );

       // Turn off hardware flow control.
       serial_port.SetFlowControl( FlowControl::FLOW_CONTROL_NONE );

       // Disable parity.
       serial_port.SetParity( Parity::PARITY_NONE );
       
       // Set the number of stop bits.
       serial_port.SetNumberOfStopBits( StopBits::STOP_BITS_1 );
       
       // Wait for some data to be available at the serial port.
       while(!serial_port.IsDataAvailable()) 
       {
           usleep(100);
       }

       // Keep reading data from serial port and print it to the screen.
       while(serial_port.IsDataAvailable()) 
       {
           unsigned char nextByte;
           unsigned int msTimeout = 50;

           serial_port.ReadByte(nextByte, msTimeout);
           std::cerr << std::hex << static_cast<int>(nextByte) << " ";
           usleep(100);
       }

       std::cerr << std::endl;
       return EXIT_SUCCESS;
   }


Complete Example Programs:
Writing to a SerialPort:
-------------------------

.. code-block:: c++

   #include <SerialPort.h>
   #include <iostream>
   #include <fstream>
   #include <cstdlib>

   using namespace LibSerial;

   // This example reads the contents of a file and writes the entire 
   // file to the serial port one character at a time.
   int main(int argc, char** argv)
   {
       if (argc < 2) 
       {
           std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
           return 1;
       }
       
       // Instantiate a SerialPort object and open the serial port.
       SerialPort serial_port;

       serial_port.Open("/dev/ttyUSB1");

       // Check that the serial stream has opened correctly.
       if (!serial_port.IsOpen()) 
       {
           std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                     << "Error: Could not open serial port." 
                     << std::endl;
           exit(1);
       }

       // Set the baud rate of the serial port.
       serial_port.SetBaudRate( BaudRate::BAUD_115200 );

       // Set the number of data bits.
       serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );

       // Turn off hardware flow control.
       serial_port.SetFlowControl( FlowControl::FLOW_CONTROL_NONE );

       // Disable parity.
       serial_port.SetParity( Parity::PARITY_NONE );
       
       // Set the number of stop bits.
       serial_port.SetNumberOfStopBits( StopBits::STOP_BITS_1 );

       // Open the input file for reading. 
       std::ifstream input_file( argv[1] );
       
       if (!input_file.good()) 
       {
           std::cerr << "Error: Could not open file "
                     << argv[1] << " for reading." << std::endl;
           return 1 ;
       }

       // Read characters from the input file and dump them to the serial port. 
       std::cerr << "Dumping file to serial port." << std::endl;
       
       while ( input_file ) 
       {
           char readByte;
           unsigned char writeByte;

           input_file.read( &readByte, 1 );

           writeByte = readByte;

           serial_port.WriteByte( writeByte );

           // Print a '.' for every character read from the input file. 
           std::cerr << ".";
       }

       std::cerr << std::endl;
       std::cerr << "Done." << std::endl;
       return EXIT_SUCCESS;
   }
