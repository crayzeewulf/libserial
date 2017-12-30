/******************************************************************************
 *   @file SerialPort.h                                                       *
 *   @copyright (C) 2004 Manish Pagey                                         *
 *   crayzeewulf@users.sourceforge.net                                        *
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

#ifndef _SerialPort_h_
#define _SerialPort_h_

#include "SerialPortConstants.h"

#include <memory>

namespace LibSerial 
{
    /**
     * @brief SerialPort allows an object oriented approach to serial port
     *        communication.  A serial port object can be created to
     *        allow opening the port with specified modes and settings.
     *        The SerialPort class also provides Get/Set methods to
     *        access the most commonly utilized parameters associated
     *        with serial port communication.
     *
     */
    class SerialPort
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit SerialPort();

        /**
         * @brief Constructor that allows a SerialPort instance to be 
         *        created and opened, initializing the corresponding
         *        serial port with the specified parameters.
         * @param fileName The file name of the serial port.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial port.
         * @param stopBits The number of stop bits for the serial port.
         * @param flowControlType The flow control type for the serial port.
         */
        explicit SerialPort(const std::string&   fileName,
                            const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                            const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                            const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                            const Parity&        parityType      = Parity::PARITY_DEFAULT,
                            const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT);

        /**
         * @brief Default Destructor for a SerialPort object. Closes the
         *        serial port associated with mFileDescriptor if open.
         */
        virtual ~SerialPort() noexcept;

        /**
         * @brief Opens the serial port associated with the specified
         *        file name and the specified mode.
         * @param fileName The file name of the serial port.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName,
                  const std::ios_base::openmode& openMode = std::ios_base::in | std::ios_base::out);

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Waits until the write buffer is drained and then returns.
         */
        void DrainWriteBuffer();

        /**
         * @brief Flushes the serial port input buffer.
         */
        void FlushInputBuffer();

        /**
         * @brief Flushes the serial port output buffer.
         */
        void FlushOutputBuffer();

        /**
         * @brief Flushes the serial port input and output buffers.
         */
        void FlushIOBuffers();

        /**
         * @brief Checks if data is available at the input of the serial port.
         * @return Returns true iff data is available to read.
         */
        bool IsDataAvailable();

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen();

        /**
         * @brief Sets all serial port paramters to their default values.
         */
        void SetDefaultSerialPortParameters();

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
         * @brief Gets the current flow control setting.
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
         * @param stopBits The number of stop bits to set.
         */
        void SetStopBits(const StopBits& stopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetStopBits();

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @note See VMIN in man termios(3).
         * @param vmin the number of minimum characters to be set.
         */
        void SetVMin(const short vmin);

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
        void SetVTime(const short vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

        /**
         * @brief Sets the DTR line to the specified value.
         * @param dtrState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetDTR(const bool dtrState = true);

        /**
         * @brief Gets the status of the DTR line.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDTR();

        /**
         * @brief Set the RTS line to the specified value.
         * @param rtsState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetRTS(const bool rtsState = true);

        /**
         * @brief Get the status of the RTS line.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS();

        /**
         * @brief Get the status of the CTS line.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCTS();

        /**
         * @brief Get the status of the DSR line.
         * @return Returns true iff the status of the DSR line is high.
         */
        bool GetDSR();

        /**
         * @brief Gets the serial port file descriptor.
         * @return Returns the serial port file descriptor.
         */
        int GetFileDescriptor();

        /**
         * @brief Gets the number of bytes available in the read buffer.
         * @return Returns the number of bytes avilable in the read buffer.
         */
        int GetNumberOfBytesAvailable();

        /**
         * @brief Gets a list of available serial ports.
         * @return Returns a std::vector of std::strings with the name of
         *         each available serial port. 
         */
        std::vector<std::string> GetAvailableSerialPorts();

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout). If msTimeout
         *        is zero, then the method will block until all requested bytes
         *        are received. If numberOfBytes is zero and msTimeout is
         *        non-zero,  the method will continue receiving data for the
         *        specified of milliseconds. If numberOfBytes is zero and
         *        msTimeout is zero, the method will return immediately. In all
         *        cases, any data received remains available in the dataBuffer
         *        on return from this method.
         * @param dataBuffer The data buffer to place data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(DataBuffer&  dataBuffer,
                  const size_t numberOfBytes = 0,
                  const size_t msTimeout = 0);

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout). If msTimeout
         *        is zero, then the method will block until all requested bytes
         *        are received. If numberOfBytes is zero and msTimeout is
         *        non-zero,  the method will continue receiving data for the
         *        specified of milliseconds. If numberOfBytes is zero and
         *        msTimeout is zero, the method will return immediately. In all
         *        cases, any data received remains available in the dataString
         *        on return from this method.
         * @param dataString The string to place data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(std::string& dataString,
                  const size_t numberOfBytes = 0,
                  const size_t msTimeout = 0);

        /**
         * @brief Reads a single byte from the serial port. If no data is 
         *        available within the specified number of milliseconds,
         *        (msTimeout), then this method will throw a ReadTimeout
         *        exception. If msTimeout is zero, then this method will
         *        block until data becomes available.
         * @param charBuffer The character read from the serial port.
         * @param msTimeout The timeout period in milliseconds.
         */
        void ReadByte(char&        charBuffer,
                      const size_t msTimeout = 0);

        /**
         * @brief Reads a single byte from the serial port. If no data is 
         *        available within the specified number of milliseconds,
         *        (msTimeout), then this method will throw a ReadTimeout
         *        exception. If msTimeout is zero, then this method will
         *        block until data becomes available.
         * @param charBuffer The character read from the serial port.
         * @param msTimeout The timeout period in milliseconds.
         */
        void ReadByte(unsigned char& charBuffer,
                      const size_t   msTimeout = 0);

        /**
         * @brief Reads a line of characters from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout).
         *        If msTimeout is 0, then this method will block until a line
         *        terminator is received. In all cases, any data received
         *        remains available in the string on return from this method.
         * @param dataString The data string read from the serial port.
         * @param lineTerminator The line termination character to specify the
         *        end of a line.
         * @param msTimeout The timeout value to return if a line termination
         *        character is not read.
         */
        void ReadLine(std::string&  dataString,
                      const char    lineTerminator = '\n',
                      const size_t  msTimeout = 0);

        /**
         * @brief Writes a DataBuffer to the serial port.
         * @param dataBuffer The DataBuffer to write to the serial port.
         */
        void Write(const DataBuffer& dataBuffer);

        /**
         * @brief Writes a std::string to the serial port.
         * @param dataString The data string to write to the serial port.
         */
        void Write(const std::string& dataString);

        /**
         * @brief Writes a single byte to the serial port.
         * @param charbuffer The byte to write to the serial port.
         */
        void WriteByte(const char charbuffer);

        /**
         * @brief Writes a single byte to the serial port.
         * @param charbuffer The byte to write to the serial port.
         */
        void WriteByte(const unsigned char charbuffer);


    protected:

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
        SerialPort& operator=(const SerialPort& otherSerialPort) = delete;

        /**
         * @brief Move assignment is not allowed.
         */
        SerialPort& operator=(const SerialPort&& otherSerialPort) = delete;

        /**
         * @brief Forward declaration of the Implementation class folowing
         *        the PImpl idiom.
         */
        class Implementation;

        /**
         * @brief Pointer to Implementation class instance.
         */
        std::unique_ptr<Implementation> mImpl;

    }; // class SerialPort

} // namespace LibSerial

#endif // #ifndef _SerialPort_h_