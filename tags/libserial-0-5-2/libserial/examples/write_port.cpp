#include <iostream>
#include <fstream>
#include <SerialStream.h>

//
// This example reads the contents of a file and writes the entire 
// file to the serial port one character at a time.
//
int
main( int    argc,
      char** argv  )
{
    //
    if ( argc < 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl ;
        return 1 ;
    }
    //
    // Open the serial port.
    //
    const char* const SERIAL_PORT_DEVICE = "/dev/ttyS0" ;
    using namespace LibSerial ;    
    SerialStream serial_port ;
    serial_port.Open( SERIAL_PORT_DEVICE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not open serial port " 
                  << SERIAL_PORT_DEVICE 
                  << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_9600 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Turn on hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Do not skip whitespace characters while reading from the
    // serial port.
    //
    // serial_port.unsetf( std::ios_base::skipws ) ;
    //
    // Open the input file for reading. 
    //
    std::ifstream input_file( argv[1] ) ;
    if ( ! input_file.good() ) 
    {
        std::cerr << "Error: Could not open file "
                  << argv[1] << " for reading." << std::endl ;
        return 1 ;
    }
    //
    // Read characters from the input file and dump them to the serial 
    // port. 
    //
    std::cerr << "Dumping file to serial port." << std::endl ;
    while( input_file ) 
    {
        char next_byte ;
        input_file.read( &next_byte, 1 ) ;
        serial_port.write( &next_byte, 1 ) ;
        //
        // Print a '.' for every character read from the input file. 
        //
        std::cerr << "." ;
    }
    std::cerr << std::endl ;
    std::cerr << "Done." << std::endl ;
    return EXIT_SUCCESS ;
}
