/***************************************************************************
 *   @file SerialPort.h                                                  *
 *   @copyright (C) 2004 by Manish Pagey                                   *
 *   crayzeewulf@users.sourceforge.net                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _SerialPort_h_
#define _SerialPort_h_

#include <string>
#include <vector>
#include <stdexcept>
#include <termios.h>


//
// :TODO: This class will be placed in LibSerial namespace in the next 
// version. 
//

/**
 * @note This class attaches a handler to the SIGIO signal to detect
 *       the data arriving at a serial port. However, this signal handler
 *       will also call any signal handler that is already attached to
 *       this signal. However, if other parts of the application attach a
 *       signal handler to SIGIO after constructing an instance of SIGIO,
 *       they must ensure that they call the existing signal handler.
 *       Otherwise, it may not be possible to receive any data through
 *       the serial port using this class.
 *
 * @todo: Provide examples of the above potential problem.
 *
 * @todo: The current implementation does not check if another process
 *        has locked the serial port device and does not lock the serial port
 *        device after opening it. This has been observed to cause problems
 *        while using this library while other programs such as minicom are
 *        also accessing the same device.  It will be useful to lock the
 *        serial port device when it is being used by this class.
 */
class SerialPort
{
public:
    /**
     * @brief The allowed set of baud rates.
     */
    enum BaudRate {
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

        // @TODO: Bug#1318912: B460800 is defined on Linux but not on Mac OS X.
        // What about other operating systems ?
#ifdef __linux__
        BAUD_460800 = B460800,
        BAUD_500000 = B500000,
        BAUD_576000 = B576000,
        BAUD_921600 = B921600,
        BAUD_1000000 = B1000000, 
        BAUD_1152000 = B1152000, 
        BAUD_1500000 = B1500000,
        BAUD_2000000 = B2000000,
#if __MAX_BAUD > B2000000
        BAUD_2500000 = B2500000,
        BAUD_3000000 = B3000000,
        BAUD_3500000 = B3500000,
        BAUD_4000000 = B4000000,
#endif
#endif /* __linux__ */
        BAUD_DEFAULT = BAUD_57600
    } ;

    /**
     * @brief The allowed set of character sizes.
     */
    enum CharacterSize
    {
        CHAR_SIZE_5 = CS5, //!< 5 bit characters.
        CHAR_SIZE_6 = CS6, //!< 6 bit characters.
        CHAR_SIZE_7 = CS7, //!< 7 bit characters.
        CHAR_SIZE_8 = CS8, //!< 8 bit characters.
        CHAR_SIZE_DEFAULT = CHAR_SIZE_8
    } ;

    /**
     * @brief The allowed number of stop bits.
     */
    enum StopBits
    {
        STOP_BITS_1, //! 1 stop bit.
        STOP_BITS_2, //! 2 stop bits.
        STOP_BITS_DEFAULT = STOP_BITS_1
    } ;

    /**
     * @brief The allowed parity types.
     */
    enum Parity
    {
        PARITY_EVEN,    //!< Even parity.
        PARITY_ODD,     //!< Odd parity.
        PARITY_NONE,    //!< No parity i.e. parity checking disabled.
        PARITY_DEFAULT = PARITY_NONE
    } ;

    /**
     * @brief The allowed flow control types.
     */
    enum FlowControl
    {
        FLOW_CONTROL_HARD,
        FLOW_CONTROL_SOFT,
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
    } ;

    class NotOpen : public std::logic_error
    {
    public:
        NotOpen( const std::string& whatArg )
            : logic_error( whatArg ) 
        {
        }
    } ;

    class OpenFailed : public std::runtime_error
    {
    public:
        OpenFailed( const std::string& whatArg )
            : runtime_error( whatArg )
        {
        }
    } ;

    class AlreadyOpen : public std::logic_error
    {
    public:
        AlreadyOpen( const std::string& whatArg )
            : logic_error( whatArg )
        {
        }
    } ;

    class UnsupportedBaudRate : public std::runtime_error
    {
    public:
        UnsupportedBaudRate( const std::string& whatArg )
            : runtime_error( whatArg )
        {
        }
    } ;

    class ReadTimeout : public std::runtime_error
    {
    public:
        ReadTimeout()
            : runtime_error("Read timeout")
        {
        }
    } ;

    /**
     * @brief Default Constructor for a serial port object.
     */
    explicit SerialPort( const std::string& serialPortName );

    /**
     * @brief Default Destructor for a serial port object.
     */
    virtual ~SerialPort() throw() ;

    /**
     * @brief Opens the serial port with the specified settings.
     *        A serial port cannot be used until it has been opened.
     * @throw AlreadyOpen This exception is thrown if the serial port
     *        is already open.
     * @throw OpenFailed This exception is thrown if the serial port
     *        could not be opened.
     * @throw std::invalid_argument This exception is thrown if an
     *        invalid parameter value is specified.
     */
    void Open( const BaudRate baudRate           = BAUD_DEFAULT,
               const CharacterSize characterSize = CHAR_SIZE_DEFAULT,
               const Parity parityType           = PARITY_DEFAULT,
               const StopBits stopBits           = STOP_BITS_DEFAULT,
               const FlowControl flowControl     = FLOW_CONTROL_DEFAULT )
        throw( AlreadyOpen,
               OpenFailed,
               UnsupportedBaudRate,
               std::invalid_argument) ;

    /**
     * @brief Determines if the serial port is open for I/O.
     * @return Returns true iff the serial port is open.
     */
    bool IsOpen() const ;

    /**
     * @brief Closes the serial port. All settings of the serial port will be
     *        lost and no more I/O can be performed on the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     */
    void Close()
        throw( NotOpen ) ;

    /**
     * @brief Sets the baud rate for the serial port to the specified value
     * @param baudRate The baud rate to be set for the serial port.
     * @throw UnsupportedBaudRate Thrown if an unsupported baud rate is specified.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::invalid_argument This exception is thrown if an invalid baud rate or other argument is specified.
     */
    void SetBaudRate( const BaudRate baudRate )
        throw( UnsupportedBaudRate,
               NotOpen,
               std::invalid_argument ) ;

    /**
     * @brief Gets the current baud rate for the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     *        port is not open.
     */
    BaudRate GetBaudRate() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Sets the character size for the serial port.
     * @brief characterSize the number of bytes each character is represented within.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::invalid_argument This exception is thrown if an invalid character size is specified.
     */
    void SetCharSize( const CharacterSize characterSize )
        throw( NotOpen,
               std::invalid_argument ) ;
    
    /**
     * @brief Gets the current character size for the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     */
    CharacterSize GetCharSize() const
        throw( NotOpen ) ;

    /**
     * @brief Sets the parity type for the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::invalid_argument This exception is thrown if an invalid parity is specified.
     */
    void SetParity( const Parity parityType )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * @brief Gets the parity type for the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     */
    Parity GetParity() const
        throw( NotOpen ) ;

    /**
     * @brief Sets the number of stop bits to be used with the serial port.
     * @brief numOfStopBits The number of stop bits to set.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::invalid_argument This exception is thrown if an invalid number of stop bits is specified.
     */
    void SetNumOfStopBits( const StopBits numOfStopBits )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * @brief Gets the number of stop bits currently being used by the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     */
    StopBits GetNumOfStopBits() const
        throw( NotOpen ) ;

    /**
     * @brief Sets flow control for the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::invalid_argument This exception is thrown if an invalid flow control is specified.
     */
    void SetFlowControl( const FlowControl flowControl )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * @brief Get the current flow control setting.
     * @throw NotOpen This exception is thrown if the method is called while the serial port is not open.
     * @return Returns the flow control type of the serial port.
     */
    FlowControl GetFlowControl() const
        throw( NotOpen ) ;

    /**
     * @brief Checks if data is available at the input of the serial port.
     * @throw NotOpen This exception is thrown if the method is called while the serial port is not open.
     * @return Returns true iff data is available to read.
     */
    bool IsDataAvailable() const
        throw( NotOpen ) ;

    /**
     * @brief Reads the specified number of bytes from the serial port.
     *        The method will timeout if no data is received in the specified
     *        number of milliseconds (msTimeout). If msTimeout is 0, then
     *        this method will block till all requested bytes are
     *        received. If numberOfBytes is zero, then this method will keep
     *        reading data till no more data is available at the serial port.
     *        In all cases, all read data is available in dataBuffer on
     *        return from this method.
     * @param dataBuffer The data buffer to place serial data into.
     * @param numberOfBytes The number of bytes to read before returning.
     * @param msTimeout The timeout period in milliseconds.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw ReadTimeout This exception is thrown if the timeout value is reached before a line termination character is received.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     * @throw NotOpen
     */

    typedef std::vector<unsigned char> DataBuffer;

    void Read( DataBuffer& dataBuffer,
               const unsigned int numberOfBytes = 0,
               const unsigned int msTimeout = 0 )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;

    /**
     * @brief Reads a single byte from the serial port.
     *        If no data is available within the specified number
     *        of milliseconds (msTimeout), then this method will
     *        throw a ReadTimeout exception. If msTimeout is 0,
     *        then this method will block until data is available.
     * @param msTimeout The timeout period in milliseconds.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw ReadTimeout This exception is thrown if the timeout value is reached before a line termination character is received.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     * @return Returns the byte read.
     */
    unsigned char ReadByte( const unsigned int msTimeout = 0 )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;

    /**
     * @brief Reads a line of characters from the serial port.
     * @param msTimeout The timeout value to return if a line termination character is not read.
     * @param lineTerminator The line termination character to specify the end of a line.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw ReadTimeout This exception is thrown if the timeout value is reached before a line termination character is received.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     * @return Returns the line read from the serial port ending with the line termination character.
     */
    const std::string ReadLine( const unsigned int msTimeout = 0,
                                const char lineTerminator = '\n' )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;

    /**
     * @brief Writes the dataBuffer vector to the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    void Write( const DataBuffer& dataBuffer )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Writes a std::string to the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    void Write( const std::string& dataString )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Writes a single byte to the serial port.
     * @param dataByte The byte to be written to the serial port.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    void WriteByte( const unsigned char dataByte )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Sets the DTR line to the specified value.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    void SetDtr( const bool dtrState = true )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Gets the status of the DTR line.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    bool GetDtr() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Sets the RTS (ready-to-send) line to the specified value.
     * @param rtsState The RTS line state to set.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    void SetRts( const bool rtsState = true )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Gets the status of the RTS (ready-to-send) line.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    bool GetRts() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Gets the status of the CTS (clear-to-send) line.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    bool GetCts() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * @brief Gets the status of the DSR (data-set-ready) line.
     * @throw NotOpen This exception is thrown if this method is called while the serial port is not open.
     * @throw std::runtime_error This exception is thrown if any standard runtime error is encountered.
     */
    bool GetDsr() const
        throw( NotOpen,
               std::runtime_error ) ;

private:
    /**
     * @brief Prevents copying of objects of this class by declaring the copy
     *        constructor private. This method is never defined.
     */
    SerialPort( const SerialPort& otherSerialPort );

    /**
     * @brief Prevents copying of objects of this class by declaring the assignment
     *        operator private. This method is never defined.
     */
    SerialPort& operator=( const SerialPort& otherSerialPort );

    /**
     * @brief Forward declaration of the implementation class folowing the PImpl idiom.
     */
    class SerialPortImpl;

    /**
     * @brief Pointer to implementation class instance.
     */
    SerialPortImpl* mSerialPortImpl;
};

#endif // #ifndef _SerialPort_h_
