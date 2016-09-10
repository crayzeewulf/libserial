#include <SerialStream.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace LibSerial;

// This example reads the contents of a file and writes the entire 
// file to the serial port one character at a time.
int main(int argc, char** argv)
{
    (void) argc; // Quiet compiler warning.
    (void) argv; // Quiet compiler warning.

    if (argc < 2) 
    {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
        return 1;
    }

    // Open the serial port.
    const char* const SERIAL_PORT_DEVICE = "/dev/ttyUSB0";
    
    SerialStream serialStream;
    serialStream.Open(SERIAL_PORT_DEVICE);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not open serial port " 
                  << SERIAL_PORT_DEVICE 
                  << std::endl;
        exit(1);
    }

    // Set the baud rate of the serial port.
    serialStream.SetBaudRate(SerialStreamBuf::BAUD_115200);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl;
        exit(1);
    }

    // Set the number of data bits.
    serialStream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl;
        exit(1);
    }

    // Disable parity.
    serialStream.SetParity(SerialStreamBuf::PARITY_NONE);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl;
        exit(1);
    }

    // Set the number of stop bits.
    serialStream.SetNumOfStopBits(1);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl;
        exit(1);
    }

    // Turn on hardware flow control.
    serialStream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl;
        exit(1);
    }

    // Do not skip whitespace characters while reading from the serial port.
    // serialStream.unsetf(std::ios_base::skipws);

    // Open the input file for reading. 
    std::ifstream input_file(argv[1]);
    
    if (!input_file.good()) 
    {
        std::cerr << "Error: Could not open file "
                  << argv[1] << " for reading." << std::endl;
        return 1;
    }

    // Read characters from the input file and dump them to the serial port. 
    std::cerr << "Dumping file to serial port." << std::endl;
    
    while(input_file) 
    {
        char nextByte;
        input_file.read( &nextByte, 1 );
        serialStream.write( &nextByte, 1 );

        // Print a '.' for every character read from the input file. 
        std::cerr << ".";
    }

    std::cerr << std::endl;
    std::cerr << "Done." << std::endl;
    return EXIT_SUCCESS;
}
