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
