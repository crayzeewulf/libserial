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

#include <string>
#include <vector>
#include <stdexcept>
#include <memory>
#include <cstdint>
#include <termios.h>


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
        using DataBuffer = std::vector<uint8_t> ;

        /**
         * @brief The allowed set of baud rates.
         */
        enum BaudRate
        {
            BAUD_50      = B50,
            BAUD_75      = B75,
            BAUD_110     = B110,
            BAUD_134     = B134,
            BAUD_150     = B150,
            BAUD_200     = B200,
            BAUD_300     = B300,
            BAUD_600     = B600,
            BAUD_1200    = B1200,
            BAUD_1800    = B1800,
            BAUD_2400    = B2400,
            BAUD_4800    = B4800,
            BAUD_9600    = B9600,
            BAUD_19200   = B19200,
            BAUD_38400   = B38400,
            BAUD_57600   = B57600,
            BAUD_115200  = B115200,
            BAUD_230400  = B230400,
            //
            // Bug#1318912: B460800 is defined on Linux but not on Mac OS
            // X. What about other operating systems ?
            //
#ifdef __linux__
            BAUD_460800 = B460800,
            BAUD_500000 = B500000,
            BAUD_576000 = B576000,
            BAUD_921600 = B921600,
            BAUD_1000000 = B1000000, 
            BAUD_1152000 = B1152000, 
            BAUD_1500000 = B1500000,
            BAUD_2000000 = B2000000,
            BAUD_2500000 = B2500000,
            BAUD_3000000 = B3000000,
            BAUD_3500000 = B3500000,
            BAUD_4000000 = B4000000,
#endif
            BAUD_DEFAULT = BAUD_57600
        };

        /**
         * @brief The allowed set of character sizes.
         */
        enum CharacterSize
        {
            CHAR_SIZE_5  = CS5, //!< 5 bit characters.
            CHAR_SIZE_6  = CS6, //!< 6 bit characters.
            CHAR_SIZE_7  = CS7, //!< 7 bit characters.
            CHAR_SIZE_8  = CS8, //!< 8 bit characters.
            CHAR_SIZE_DEFAULT = CHAR_SIZE_8
        };

        /**
         * @brief The allowed number of stop bits.
         */
        enum StopBits
        {
            STOP_BITS_1,   //! 1 stop bit.
            STOP_BITS_2,   //! 2 stop bits.
            STOP_BITS_DEFAULT = STOP_BITS_1
        };

        /**
         * @brief The allowed parity types.
         */
        enum Parity
        {
            PARITY_EVEN,     //!< Even parity.
            PARITY_ODD,      //!< Odd parity.
            PARITY_NONE,     //!< No parity i.e. parity checking disabled.
            PARITY_DEFAULT = PARITY_NONE
        };

        /**
         * @brief The allowed flow control types.
         */
        enum FlowControl
        {
            FLOW_CONTROL_HARD,
            FLOW_CONTROL_SOFT,
            FLOW_CONTROL_NONE,
            FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
        };

        class NotOpen : public std::logic_error
        {
        public:
            NotOpen(const std::string& whatArg)
                : logic_error(whatArg)
            {
            }
        };

        class OpenFailed : public std::runtime_error
        {
        public:
            OpenFailed(const std::string& whatArg)
                : runtime_error(whatArg)
            {
            }
        };

        class AlreadyOpen : public std::logic_error
        {
        public:
            AlreadyOpen( const std::string& whatArg)
                : logic_error(whatArg)
            {
            }
        };

        class UnsupportedBaudRate : public std::runtime_error
        {
        public:
            UnsupportedBaudRate( const std::string& whatArg )
                : runtime_error(whatArg)
            {
            }
        };

        class ReadTimeout : public std::runtime_error
        {
        public:
            ReadTimeout()
                : runtime_error("Read timeout")
            {
            }
        };

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
         * @param parityType The serial port parity type.
         * @param stopBits The serial port number of stop bits.
         * @param flowControl The serial port flow control type.
         */
        void Open(const BaudRate      baudRate    = BAUD_DEFAULT,
                  const CharacterSize charSize    = CHAR_SIZE_DEFAULT,
                  const Parity        parityType  = PARITY_DEFAULT,
                  const StopBits      stopBits    = STOP_BITS_DEFAULT,
                  const FlowControl   flowControl = FLOW_CONTROL_DEFAULT);

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen() const;

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Sets the baud rate for the serial port to the specified value
         * @param baudRate The baud rate to be set for the serial port.
         */
        void SetBaudRate(const BaudRate baudRate);

        /**
         * @brief Gets the current baud rate for the serial port.
         * @return Returns the baud rate.
         */
        BaudRate GetBaudRate() const;

        /**
         * @brief Sets the character size for the serial port.
         * @param charSize The character size to be set.
         */
        void SetCharSize(const CharacterSize charSize);

        /**
         * @brief Gets the current character size for the serial port.
         * @return Returns the character size.
         */
        CharacterSize GetCharSize() const;

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity parityType);

        /**
         * @brief Gets the parity type for the serial port.
         * @return Returns the parity type.
         */
        Parity GetParity() const;

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param numOfStopBits The number of stop bits to set.
         */
        void SetNumOfStopBits(const StopBits numOfStopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetNumOfStopBits() const;

        /**
         * @brief Sets flow control for the serial port.
         * @param flowControl The flow control type to be set.
         */
        void SetFlowControl(const FlowControl flowControl);

        /**
         * @brief Get the current flow control setting.
         * @return Returns the flow control type of the serial port.
         */
        FlowControl GetFlowControl() const;

        /**
         * @brief Checks if data is available at the input of the serial port.
         * @return Returns true iff data is available to read.
         */
        bool IsDataAvailable() const;

        /**
         * @brief Reads a single byte from the serial port.
         *        If no data is available within the specified number
         *        of milliseconds (msTimeout), then this method will
         *        throw a ReadTimeout exception. If msTimeout is 0,
         *        then this method will block until data is available.
         * @param msTimeout The timeout period in milliseconds.
         * @return Returns the byte read.
         */
        uint8_t ReadByte(const unsigned int msTimeout = 0);

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
         */
        void Read(DataBuffer& dataBuffer,
                  const unsigned int numOfBytes = 0,
                  const unsigned int msTimeout  = 0);

        /**
         * @brief Reads a line of characters from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until a line terminator is received.
         *        If a line terminator is read, a string will be returned,
         *        however, if the timeout is reached, an exception will be thrown
         *        and all previously read data will be lost.
         * @param msTimeout The timeout value to return if a line termination
         *        character is not read.
         * @param lineTerminator The line termination character to specify the
         *        end of a line.
         * @return Returns the line read from the serial port ending along with the
         *         line termination character iff sucessful.
         */
        std::string ReadLine(const unsigned int msTimeout = 0,
                             const char lineTerminator = '\n');

        /**
         * @brief Writes a single byte to the serial port.
         * @param dataByte The byte to be written to the serial port.
         */
        void WriteByte(const uint8_t dataByte);

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
        void SetDtr(const bool dtrState = true);

        /**
         * @brief Gets the status of the DTR line.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDtr() const;

        /**
         * @brief Set the RTS line to the specified value.
         * @param rtsState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetRts(const bool rtsState = true);

        /**
         * @brief Get the status of the RTS line.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRts() const;

        /**
         * @brief Get the status of the CTS line.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCts() const;

        /**
         * @brief Get the status of the DSR line.
         * @return Returns true iff the status of the DSR line is high.
         */
        bool GetDsr() const;
    private:
        /**
         * @brief Prevents copying of objects of this class by declaring the copy
         *        constructor private. This method is never defined.
         */
        SerialPort(const SerialPort& otherSerialPort) = delete ;

        /**
         * @brief Move construction is disallowed.
         */
        SerialPort(const SerialPort&& otherSerialPort) = delete ;

        /**
         * @brief Prevents copying of objects of this class by declaring the
         *        assignment operator private. This method is never defined.
         */
        SerialPort& operator=(const SerialPort& otherSerialPort ) = delete ;

        /**
         * @brief Move assignment is not allowed.
         */
        SerialPort& operator=(const SerialPort&& otherSerialPort ) = delete ;

        /**
         * @brief Forward declaration of the implementation class folowing
         *        the PImpl idiom.
         */
        class Implementation ;

        /**
         * @brief Pointer to implementation class instance.
         */
        std::unique_ptr<Implementation> mImpl ;
    } ;
}

#endif // #ifndef _SerialPort_h_
