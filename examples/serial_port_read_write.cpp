/******************************************************************************
 *   @file serial_port_read_write.cpp                                         *
 *   @copyright (C) 2016 LibSerial Development Team                           *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU Lessser General Public License as          *
 *   published by the Free Software Foundation; either version 2 of the       *
 *   License, or (at your option) any later version.                          *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU Lesser General Public License for more details.                      *
 *                                                                            *
 *   You should have received a copy of the GNU Lesser General Public         *
 *   License along with this program; if not, write to the                    *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#include <SerialPort.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace LibSerial;

/**
 * @brief This example demonstrates multiple methods to read and write
 *        serial stream data.
 */
int main()
{
    // Instantiate two SerialPort objects.
    SerialPort serial_port_1;
    SerialPort serial_port_2;

    // Open the Serial Ports at the desired hardware devices.
    serial_port_1.Open("/dev/ttyUSB0");
    serial_port_2.Open("/dev/ttyUSB1");

    // Verify that the serial ports opened.
    if (!serial_port_1.IsOpen() ||
        !serial_port_2.IsOpen())
    {
        std::cerr << "The serial ports did not open correctly." << std::endl;
        return EXIT_FAILURE;
    }

    // Set the baud rates.
    serial_port_1.SetBaudRate(BaudRate::BAUD_115200);
    serial_port_2.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port_1.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port_2.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Set the hardware flow control.
    serial_port_1.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port_2.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Set the parity.
    serial_port_1.SetParity(Parity::PARITY_NONE);
    serial_port_2.SetParity(Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_port_1.SetNumberOfStopBits(StopBits::STOP_BITS_1);
    serial_port_2.SetNumberOfStopBits(StopBits::STOP_BITS_1);

    // Setup variables to store outgoing and incoming data.
    char write_byte_1 = 'a';
    char write_byte_2 = 'b';

    char read_byte_1  = ' ';
    char read_byte_2  = ' ';

    // Variables to store outgoing and incoming data.
    std::string write_string_1 = "\"Do what you can, with what you have, where you are.\" - Theodore Roosevelt";
    std::string write_string_2 = "\"Simplicity is prerequisite for reliability.\" - Edsger W. Dijkstra";

    std::string read_string_1 = "";
    std::string read_string_2 = "";

    // Print to the terminal what will take place next.
    std::cout << "\nUsing WriteByte() and ReadByte() for one byte of data:"
              << std::endl;

    // Write a single byte of data to the serial ports.
    serial_port_1.WriteByte(write_byte_1);
    serial_port_2.WriteByte(write_byte_2);

    // Specify a read timeout value in milliseconds.
    size_t timeout_milliseconds = 25;

    // Read a single byte of data from the serial ports.
    serial_port_1.ReadByte(read_byte_1, timeout_milliseconds);
    serial_port_2.ReadByte(read_byte_2, timeout_milliseconds);

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_byte_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_byte_2  << std::endl
              << std::endl;

    std::cout << "\tSerial Port 2 sent:\t"     << write_byte_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_byte_1  << std::endl
              << std::endl;

    // Print to the terminal what will take place next.
    std::cout << "Using Write() and Read() for a specified number of "
              << "bytes of data:" << std::endl;

    // Write a string to each serial port.
    serial_port_1.Write(write_string_1);
    serial_port_2.Write(write_string_2);

    // Read the appropriate number of bytes from each serial port.
    serial_port_1.Read(read_string_1, write_string_2.size(), timeout_milliseconds);
    serial_port_2.Read(read_string_2, write_string_1.size(), timeout_milliseconds);

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
        std::cout << "Using Write() and ReadLine() to write a string and "
                  << "read a line of data:" << std::endl;

        // Write the user input to the serial port.
        serial_port_1.Write(user_input + "\n");

        // Read the data transmitted from the corresponding serial port.
        serial_port_2.ReadLine(read_string_2);

        // Print to the terminal what was sent and what was received.
        std::cout << "\tSerial Port 1 sent:\t"     << user_input   << std::endl
                  << "\tSerial Port 2 received:\t" << read_string_2 << std::endl;
    }

    // Close the serial ports and end the program.
    serial_port_1.Close();
    serial_port_2.Close();

    std::cout << "The example program successfully completed!" << std::endl;
    return EXIT_SUCCESS;
}
