/******************************************************************************
 *   @file SerialPort.h                                                       *
 *   @copyright (C) 2004 by Manish Pagey                                      *
 *   crayzeewulf@users.sourceforge.net                                        *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU General Public License as published by     *
 *   the Free Software Foundation; either version 2 of the License, or        *
 *   (at your option) any later version.                                      *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU General Public License for more details.                             *
 *                                                                            *
 *   You should have received a copy of the GNU General Public License        *
 *   along with this program; if not, write to the                            *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#ifndef _SerialPort_h_
#define _SerialPort_h_

#include "SerialPortConstants.h"

#include <memory>
#include <vector>

namespace LibSerial 
{
    /**
     * @note This class attaches a handler to the SIGIO signal to detect
     * the data arriving at a serial port. However, this signal handler
     * will also call any signal handler that is already attached to
     * this signal. However, if other parts of the application attach a
     * signal handler to SIGIO after constructing an instance of SIGIO,
     * they must ensure that they call the existing signal handler.
     * Otherwise, it may not be possible to receive any data through
     * the serial port using this class.
     *
     * @FIXME: Provide examples of the above potential problem.
     *
     * @todo The current implementation does not check if another process
     * has locked the serial port device and does not lock the serial port
     * device after opening it. This has been observed to cause problems
     * while using this library while other programs such as minicom are
     * also accessing the same device.  It will be useful to lock the
     * serial port device when it is being used by this class.
     */
    class SerialPort
    {
    public:

        /**
         * @brief Type used to receive and return raw data to/from methods.
         */
        using DataBuffer = std::vector<uint8_t>;

        /**
         * @brief Default Constructor for a serial port object.
         */
        explicit SerialPort(const std::string& serialPortName);

        /**
         * @brief Default Destructor for a serial port object.
         */
        virtual ~SerialPort() noexcept;

        /**
         * @brief Opens the serial port with the specified settings.
         *        A serial port cannot be used until it has been opened.
         * @param baudRate The serial port baud rate.
         * @param charSize The serial port character size.
         * @param flowControl The serial port flow control type.
         * @param parityType The serial port parity type.
         * @param stopBits The serial port number of stop bits.
         */
        void Open(const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                  const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                  const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                  const Parity&        parityType      = Parity::PARITY_DEFAULT,
                  const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT);

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen();

        /** 
         * @brief This routine is called by open() in order to
         *        initialize some parameters of the serial port and
         *        setting its parameters to default values.
         * @return -1 on failure and some other value on success. 
         */
        int InitializeSerialPort();

        /**
         * @brief Initializes the serial communication parameters to their
         *        default values.
         */
        void SetParametersToDefault();

        /**
         * @brief Sets the baud rate for the serial port to the specified value
         * @param baudRate The baud rate to be set for the serial port.
         */
        void SetBaudRate(const BaudRate& baudRate);

        /**
         * @brief Gets the current baud rate for the serial port.
         * @return Returns the baud rate.
         */
        BaudRate GetBaudRate();

        /**
         * @brief Sets the character size for the serial port.
         * @param characterSize The character size to be set.
         */
        void SetCharacterSize(const CharacterSize& characterSize);

        /**
         * @brief Gets the character size being used for serial communication.
         * @return Returns the current character size. 
         */
        CharacterSize GetCharacterSize();

        /**
         * @brief Sets flow control for the serial port.
         * @param flowControlType The flow control type to be set.
         */
        void SetFlowControl(const FlowControl& flowControlType);

        /**
         * @brief Get the current flow control setting.
         * @return Returns the flow control type of the serial port.
         */
        FlowControl GetFlowControl();

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity& parityType);

        /**
         * @brief Gets the parity type for the serial port.
         * @return Returns the parity type.
         */
        Parity GetParity();

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param numberOfStopBits The number of stop bits to set.
         */
        void SetNumberOfStopBits(const StopBits& numberOfStopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetNumberOfStopBits();

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @note See VMIN in man termios(3).
         * @param vMin the number of minimum characters to be set.
         * @return Returns the minimum number of charcters set.
         */
        void SetVMin(const short& vmin);

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for
         *         non-canonical reads.
         */
        short GetVMin();

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds.
         */
        void SetVTime(const short& vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

        /**
         * @brief Checks if data is available at the input of the serial port.
         * @return Returns true iff data is available to read.
         */
        bool IsDataAvailable();

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataBuffer on
         *        return from this method.
         * @param dataBuffer The data buffer to place serial data into.
         * @param numOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         * @return Returns the number of bytes read.
         */
        int Read(DataBuffer&         dataBuffer,
                 const unsigned int& numOfBytes = 0,
                 const unsigned int& msTimeout  = 0);

        /**
         * @brief Reads a single byte from the serial port.
         *        If no data is available within the specified number
         *        of milliseconds (msTimeout), then this method will
         *        throw a ReadTimeout exception. If msTimeout is 0,
         *        then this method will block until data is available.
         * @param msTimeout The timeout period in milliseconds.
         * @return Returns the number of bytes read.
         */
        int ReadByte(unsigned char&      charBuffer,
                     const unsigned int& msTimeout = 0);

        /**
         * @brief Reads a line of characters from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until a line terminator is received.
         *        If a line terminator is read, a string will be returned,
         *        however, if the timeout is reached, an exception will be thrown
         *        and all previously read data will be lost.
         * @param dataString The data string read from the serial port.
         * @param lineTerminator The line termination character to specify the
         *        end of a line.
         * @param msTimeout The timeout value to return if a line termination
         *        character is not read.
         * @return Returns the number of bytes read.
         */
        int ReadLine(std::string&        dataString,
                     const char&         lineTerminator = '\n',
                     const unsigned int& msTimeout = 0);

        /**
         * @brief Writes a single byte to the serial port.
         * @param dataByte The byte to be written to the serial port.
         */
        void WriteByte(const unsigned char& dataByte);

        /**
         * @brief Writes a DataBuffer vector to the serial port.
         * @param dataBuffer The DataBuffer vector to be written to the serial
         *        port.
         */
        void Write(const DataBuffer& dataBuffer);

        /**
         * @brief Writes a std::string to the serial port.
         * @param dataString The data string to be written to the serial port.
         * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
         * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
         */
        void Write(const std::string& dataString);

        /**
         * @brief Sets the DTR line to the specified value.
         * @param dtrState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetDtr(const bool& dtrState = true);

        /**
         * @brief Gets the status of the DTR line.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDtr();

        /**
         * @brief Set the RTS line to the specified value.
         * @param rtsState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetRts(const bool& rtsState = true);

        /**
         * @brief Get the status of the RTS line.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRts();

        /**
         * @brief Get the status of the CTS line.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCts();

        /**
         * @brief Get the status of the DSR line.
         * @return Returns true iff the status of the DSR line is high.
         */
        bool GetDsr();

        /**
         * @brief Gets the serial port file descriptor.
         */
        int GetFileDescriptor();

    private:
        /**
         * @brief Prevents copying of objects of this class by declaring the copy
         *        constructor private. This method is never defined.
         */
        SerialPort(const SerialPort& otherSerialPort) = delete;

        /**
         * @brief Move construction is disallowed.
         */
        SerialPort(const SerialPort&& otherSerialPort) = delete;

        /**
         * @brief Prevents copying of objects of this class by declaring the
         *        assignment operator private. This method is never defined.
         */
        SerialPort& operator=(const SerialPort& otherSerialPort ) = delete;

        /**
         * @brief Move assignment is not allowed.
         */
        SerialPort& operator=(const SerialPort&& otherSerialPort ) = delete;

        /**
         * @brief Forward declaration of the implementation class folowing
         *        the PImpl idiom.
         */
        class Implementation;

        /**
         * @brief Pointer to implementation class instance.
         */
        std::unique_ptr<Implementation> mImpl;
    };
}

#endif // #ifndef _SerialPort_h_
