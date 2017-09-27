/******************************************************************************
 *   @file serial_stream_read.cpp                                             *
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

#include <SerialStream.h>

#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace LibSerial;

/**
 * @brief This example demonstrates configuring a serial stream and 
 *        reading serial stream data.
 */
int main()
{
    // Instantiate a SerialStream object.
    SerialStream serial_stream;

    // Open the Serial Port at the desired hardware port.
    serial_stream.Open("/dev/ttyUSB0");

    // Set the baud rate of the serial port.
    serial_stream.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_stream.SetParity(Parity::PARITY_NONE);

    // Set the number of stop bits.
    serial_stream.SetNumberOfStopBits(StopBits::STOP_BITS_1);

    // Wait for data to be available at the serial port.
    while(serial_stream.rdbuf()->in_avail() == 0) 
    {
        usleep(1000);
    }

    // Variable to store data coming from the serial port.
    char data_byte;

    // String to store data for printing to terminal.
    std::string data_string;

    // Keep reading data from serial port and print it to the screen.
    while(serial_stream.IsDataAvailable()) 
    {
        // Read a single byte of data from the serial port.
        serial_stream.get(data_byte);

        // Show the user what is being read from the serial port.
        std::cout << data_byte;

        // Wait a brief period for more data to arrive.
        usleep(1000);
    }

    // Successful program completion.
    std::cout << "Done." << std::endl;
    return EXIT_SUCCESS;
}
