#include <SerialStream.h>
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
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl ;
        return 1 ;
    }

    // Open the serial port.
    const char* const SERIAL_PORT_DEVICE = "/dev/ttyUSB0" ;
    
    SerialStream serial_stream;

    serial_stream.Open(SERIAL_PORT_DEVICE);
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not open serial port " 
                  << SERIAL_PORT_DEVICE 
                  << std::endl ;
        exit(1) ;
    }

    // Set the baud rate of the serial port.
    serial_stream.SetBaudRate( BaudRate::BAUD_115200 ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }

    // Set the number of data bits.
    serial_stream.SetCharSize( CharSize::CHAR_SIZE_8 ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }

    // Disable parity.
    serial_stream.SetParity(SerialStreamBuf::PARITY_NONE);
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }

    // Set the number of stop bits.
    serial_stream.SetNumOfStopBits(1);
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }

    // Turn on hardware flow control.
    serial_stream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }

    // Do not skip whitespace characters while reading from the serial port.
    // serial_stream.unsetf(std::ios_base::skipws);

    // Open the input file for reading. 
    std::ifstream input_file( argv[1] );
    
    if ( !input_file.good() ) 
    {
        std::cerr << "Error: Could not open file "
                  << argv[1] << " for reading." << std::endl ;
        return 1 ;
    }

    // Read characters from the input file and dump them to the serial port. 
    std::cerr << "Dumping file to serial port." << std::endl ;
    
    while( input_file ) 
    {
        char nextByte;
        input_file.read( &nextByte, 1 );
        serial_stream.write( &nextByte, 1 );

        // Print a '.' for every character read from the input file. 
        std::cerr << "." ;
    }

    std::cerr << std::endl ;
    std::cerr << "Done." << std::endl ;
    return EXIT_SUCCESS ;
}
