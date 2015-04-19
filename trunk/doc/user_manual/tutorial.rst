Tutorial
========

Opening a Serial Port I/O Stream
--------------------------------

A serial port I/O stream, SerialStream, instance can be created and opened by
providing the name of the serial port device to the constructor:

.. code-block:: c++ 

   #include <SerialStream.h>
   //
   using namespace LibSerial ;
   //
   // Create and open the serial port for communication. 
   //
   SerialStream my_serial_stream( "/dev/ttyS0" ) ;

In certain applications, the name of the serial port device may not be known
when the SerialStream instance is created. In such cases, the same effect as
above can be achieved as follows:

.. code-block:: c++

   #include <SerialStream.h>
   //
   using namespace LibSerial ;
   //
   // Create a SerialStream instance.
   // 
   SerialStream my_serial_stream ;
   //
   // Open the serial port for communication.
   // 
   my_serial_stream.Open( "/dev/ttyS0" ) ;

Setting the Baud Rate
---------------------

The baud rate for the SerialStream can be set using the
SerialStream::SetBaudRate() member function.

.. code-block:: c++

   //
   // The various available baud rates are defined in SerialStreamBuf class. 
   // This is to be changed soon. All serial port settings will be placed in
   // in the SerialPort class.
   //
   my_serial_stream.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;

Setting the Character Size
--------------------------

.. code-block:: c++

   //
   // Use 8 bit wide characters. 
   //
   my_serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;

Setting the Number of Stop Bits
-------------------------------

.. code-block:: c++

   //
   // Use one stop bit. 
   //
   my_serial_port.SetNumOfStopBits(1) ;

Setting the Parity Type
-----------------------


.. code-block:: c++

   //
   // Use odd parity during serial communication. 
   // 
   my_serial_port.SetParity( SerialStreamBuf::PARITY_ODD ) ;

Setting the Flow-Control Type
-----------------------------

.. code-block:: c++ 

   // 
   // Use hardware flow-control. 
   //
   my_serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD ) ;

Reading Characters
------------------

Characters can be read from the serial port using standard iostream ">>"
operator. For example:

.. code-block:: c++ 

   //
   // Read one character from the serial port. 
   //
   char next_char ;
   my_serial_stream >> next_char ;
   //
   // You can also read other types of values from the serial port in a similar fashion. 
   //
   int data_size ;
   my_serial_stream >> data_size ;

All other methods of standard C++ iostream objects are available too. For
example, one can read characters from the serial port using the get() method:

.. code-block:: c++

   // 
   // Read one byte from the serial port. 
   //
   char next_byte ;
   my_serial_stream.get( next_byte ) ;

Writing Characters
------------------

.. code-block:: c++ 

   // 
   // Write a single character to the serial port. 
   //
   my_serial_stream << 'U' ;
   //
   // You can write a whole string. 
   //
   my_serial_stream << "Hello, Serial Port." << std::endl ;
   //
   // In fact, you can pretty much write any type of object that 
   // is supported by a "<<" operator. 
   //
   double radius = 2.0 ;
   double area = M_PI * 2.0 * 2.0 ;
   my_serial_stream << area << std::endl ;

Reading Blocks of Data
----------------------

.. code-block:: c++ 

   //
   // Read a whole array of data from the serial port. 
   //
   const int BUFFER_SIZE = 256 ;
   char input_buffer[BUFFER_SIZE] ; 
   //
   my_serial_stream.read( input_buffer, 
                          BUFFER_SIZE ) ;

Writing Blocks of Data
----------------------

.. code-block:: c++

   //
   // Write an array of data from the serial port. 
   //
   const int BUFFER_SIZE = 256 ;
   char output_buffer[BUFFER_SIZE] ; 
   //
   for(int i=0; i<BUFFER_SIZE; ++i) 
   {
       output_buffer[i] = i ;
   }
   //
   my_serial_stream.write( output_buffer, 
                           BUFFER_SIZE ) ;

Closing the Serial Port
-----------------------

.. code-block:: c++ 

   my_serial_port.Close() ;

Complete Example Programs
-------------------------

Coming soon.
