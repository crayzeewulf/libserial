#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace LibSerial;
    
int main(int argc, char** argv )
{
    (void) argc; // Quiet compiler warning.
    (void) argv; // Quiet compiler warning.

    // Instantiate the SerialStream object then open the serial port.
    SerialStream serialStream;
    serialStream.Open("/dev/ttyUSB0");
    
    if (!serialStream.good()) 
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port." 
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

    // Turn off hardware flow control.
    serialStream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!serialStream.good()) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl;
        exit(1);
    }

    // Do not skip whitespace characters while reading from the serial port.
    // serialStream.unsetf(std::ios_base::skipws);

    // Wait for some data to be available at the serial port.
    while(serialStream.rdbuf()->in_avail() == 0) 
    {
        usleep(100);
    }

    // Keep reading data from serial port and print it to the screen.
    while(serialStream.rdbuf()->in_avail() > 0) 
    {
        char nextByte;
        serialStream.get(nextByte);
        std::cerr << std::hex << static_cast<int>(nextByte) << " ";
        usleep(100);
    }

    std::cerr << std::endl;
    return EXIT_SUCCESS;
}
