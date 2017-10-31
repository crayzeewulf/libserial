/**
 *  @example serial_stream_read_write.cpp
 */

#include <SerialStream.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string.h>
#include <unistd.h>

using namespace LibSerial;

/**
 * @brief This example demonstrates multiple methods to read and write
 *        serial stream data utilizing std::iostream functionality.
 */
int main()
{
    // Instantiate two SerialStream objects.
    SerialStream serial_stream_1;
    SerialStream serial_stream_2;

    // Open the Serial Ports at the desired hardware devices.
    serial_stream_1.Open("/dev/ttyUSB0");
    serial_stream_2.Open("/dev/ttyUSB1");

    // Verify that the serial ports opened.
    if (!serial_stream_1.IsOpen() ||
        !serial_stream_2.IsOpen())
    {
        std::cerr << "The serial ports did not open correctly." << std::endl;
        return EXIT_FAILURE;
    }

    // Set the baud rates.
    serial_stream_1.SetBaudRate(BaudRate::BAUD_115200);
    serial_stream_2.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_stream_1.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_stream_2.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_stream_1.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_stream_2.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_stream_1.SetParity(Parity::PARITY_NONE);
    serial_stream_2.SetParity(Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_stream_1.SetStopBits(StopBits::STOP_BITS_1);
    serial_stream_2.SetStopBits(StopBits::STOP_BITS_1);

    // Variables to store outgoing and incoming data.
    std::string write_string_1 = "\"Do what you can, with what you have, where you are.\" - Theodore Roosevelt";
    std::string write_string_2 = "\"Simplicity is prerequisite for reliability.\" - Edsger W. Dijkstra";

    std::string read_string_1 = "";
    std::string read_string_2 = "";

    // Print to the terminal what will take place next.
    std::cout << "\nUsing write() and read() for a specified number of "
              << "bytes of data:" << std::endl;

    // Write a specified number of bytes of data.
    serial_stream_1.write(write_string_1.c_str(), write_string_1.size());
    serial_stream_2.write(write_string_2.c_str(), write_string_2.size());

    // Char arrays to store incoming data.
    char* read_array_1 = new char[write_string_2.size()];
    char* read_array_2 = new char[write_string_1.size()];

    // Use inheritted std::istream read() method to read the data.
    serial_stream_1.read(read_array_1, write_string_2.size());
    serial_stream_2.read(read_array_2, write_string_1.size());

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_string_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_array_2  << std::endl
              << std::endl;

    std::cout << "\tSerial Port 2 sent:\t"     << write_string_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_array_1  << std::endl
              << std::endl;

    // Print to the terminal what will take place next.
    std::cout << "Using the \"<<\" operator and getline() for a line of data:"
              << std::endl;

    // Write a line at each serial port.
    serial_stream_1 << write_string_1 << std::endl;
    serial_stream_2 << write_string_2 << std::endl;

    // Read a line at each serial port.
    std::getline(serial_stream_1, read_string_1);
    std::getline(serial_stream_2, read_string_2);

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_string_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_string_2  << std::endl
              << std::endl;

    std::cout << "\tSerial Port 2 sent:\t"     << write_string_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_string_1  << std::endl
              << std::endl;

    // Variable to hold user input.
    std::string user_input;
    user_input.clear();

    // Prompt the user for input.
    std::cout << "Type something you would like to send over serial,"
              << " (enter \"Q\" or \"q\" to quit): " << std::flush;

    while(true)
    {
        // Get input from the user.
        std::getline(std::cin, user_input);

        if (user_input == "q" ||
            user_input == "Q")
        {
            break;
        }

        // Print to the terminal what will take place next.
        std::cout << "Using the \"<<\" and \">>\" operators to send "
                  << "and receive your data: " << std::endl;

        // Write the user input to the serial port.
        serial_stream_1 << user_input << std::endl;

        // Read the data transmitted from the corresponding serial port.
        // Note: The ">>" operator behavior is tricky if whitespace or
        //       nothing is entered by the user!
        serial_stream_2 >> read_string_2;

        // Print to the terminal what was sent and what was received.
        std::cout << "\tSerial Port 1 sent:\t"     << user_input   << std::endl
                  << "\tSerial Port 2 received:\t" << read_string_2 << std::endl;
    }

    // Close the serial ports and end the program.
    serial_stream_1.Close();
    serial_stream_2.Close();

    std::cout << "The example program successfully completed!" << std::endl;
    return EXIT_SUCCESS;
}
