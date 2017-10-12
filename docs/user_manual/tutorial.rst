Tutorial
========

Opening a Serial Port I/O Stream
--------------------------------

A serial port instance, SerialPort, or an I/O stream instance,
SerialStream, can be created and opened by providing the name of the
serial port device to the constructor:

.. code-block:: c++ 

   #include <SerialPort.h>
   #include <SerialStream.h>

   using namespace LibSerial ;

   // Create and open the serial port for communication.
   SerialPort   my_serial_port( "/dev/ttyS0" );
   SerialStream my_serial_stream( "/dev/ttyUSB0" ) ;

In certain applications, the name of the serial port device may not be known
when the SerialStream instance is created. In such cases, the same effect as
above can be achieved as follows:

.. code-block:: c++

   // Create a object instance.
   SerialPort   my_serial_port;
   SerialStream my_serial_stream;

   // Obtain the serial port name from user input.
   std::cout << "Please enter the name of the serial device, (e.g. /dev/ttyUSB0): " << std::flush;
   std::string serial_port_name;
   std::cin >> serial_port_name;

   // Open the serial port for communication.
   my_serial_port.Open( serial_port_name );
   my_serial_stream.Open( serial_port_name );

Setting the Baud Rate
---------------------

The baud rate for the SerialStream can be set using the
SerialStream::SetBaudRate() member function.

.. code-block:: c++

   // Set the desired baud rate using a SetBaudRate() method call.
   // Available baud rate values are defined in SerialStreamConstants.h.

   my_serial_port.SetBaudRate( BAUD_115200 );
   my_serial_stream.SetBaudRate( BAUD_115200 );

Setting the Character Size
--------------------------

.. code-block:: c++

   // Set the desired character size using a SetCharacterSize() method call.
   // Available character size values are defined in SerialStreamConstants.h.
   my_serial_port.SetCharacterSize( CHAR_SIZE_8 );
   my_serial_stream.SetCharacterSize( CHAR_SIZE_8 );

Setting the Flow-Control Type
-----------------------------

.. code-block:: c++ 

   // Set the desired flow control type using a SetFlowControl() method call.
   // Available flow control types are defined in SerialStreamConstants.h.
   my_serial_port.SetFlowControl( FLOW_CONTROL_HARD );
   my_serial_stream.SetFlowControl( FLOW_CONTROL_HARD );


Setting the Parity Type
-----------------------

.. code-block:: c++

   // Set the desired parity type using a SetParity() method call.
   // Available parity types are defined in SerialStreamConstants.h.
   my_serial_port.SetParity( PARITY_ODD );
   my_serial_stream.SetParity( PARITY_ODD );


Setting the Number of Stop Bits
-------------------------------

.. code-block:: c++

   // Set the number of stop bits using a SetNumOfStopBits() method call.
   // Available stop bit values are defined in SerialStreamConstants.h.
   my_serial_port.SetNumOfStopBits( STOP_BITS_1 ) ;
   my_serial_stream.SetNumOfStopBits( STOP_BITS_1 ) ;


Reading Characters
------------------

Characters can be read from serial port instances using Read(), ReadByte(),
and Readline() methods. For example:

.. code-block:: c++ 

   // Read one character from the serial port within the timeout allowed.
   int timeout_ms = 25; // timeout value in milliseconds
   char next_char;      // variable to store the read result

   my_serial_port.ReadByte( next_char, timeout_ms );
   my_serial_stream.read( next_char );


Characters can be read from serial streams using standard iostream operators. For example:

.. code-block:: c++ 

   // Read one character from the serial port. 
   char next_char;
   my_serial_stream >> next_char;

   // You can also read other types of values from the serial port in a similar fashion. 
   int data_size;
   my_serial_stream >> data_size;

Other methods of standard C++ iostream objects could be used as well.
For example, one can read characters from the serial stream using the get() method:

.. code-block:: c++

   // Read one byte from the serial port. 
   char next_byte;
   my_serial_stream.get( next_byte );

Writing Characters
------------------

.. code-block:: c++ 

   // Write a single character to the serial port.
   my_serial_port.WriteByte( 'U' );
   my_serial_stream << 'U' ;

   // You can easily write strings.
   std::string my_string = "Hello, Serial Port." 

   my_serial_port.Write( my_string );
   my_serial_stream << my_string << std::endl ;

   // And, with serial stream objects, you can easily write any type
   // of object that is supported by a "<<" operator.
   double radius = 2.0 ;
   double area = M_PI * 2.0 * 2.0 ;

   my_serial_stream << area << std::endl ;

Reading Blocks of Data
----------------------

.. code-block:: c++ 

   // Read a whole array of data from the serial port. 
   const int BUFFER_SIZE = 256;
   char input_buffer[BUFFER_SIZE];

   my_serial_port.Read( input_buffer, BUFFER_SIZE );
   my_serial_stream.read( input_buffer, BUFFER_SIZE );

Writing Blocks of Data
----------------------

.. code-block:: c++

   // Write an array of data from the serial port. 
   const int BUFFER_SIZE = 256;
   char output_buffer[BUFFER_SIZE];

   for( int i=0; i<BUFFER_SIZE; ++i ) 
   {
       output_buffer[i] = i;
   }

   my_serial_port.Write( output_buffer, BUFFER_SIZE );
   my_serial_stream.write( output_buffer, BUFFER_SIZE );

Closing the Serial Port
-----------------------

.. code-block:: c++ 

   my_serial_port.Close();
   my_serial_stream.Close();
