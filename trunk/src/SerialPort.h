/***************************************************************************
 *   Copyright (C) 2004 by Manish Pagey                                    *
 *   crayzeewulf@users.sourceforge.net
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


/**
 *
 * @note This class attaches a handler to the SIGIO signal to detect
 * the data arriving at a serial port. However, this signal handler
 * will also call any signal handler that is already attached to
 * this signal. However, if other parts of the application attach a
 * signal handler to SIGIO after constructing an instance of SIGIO,
 * they must ensure that they call the existing signal handler.
 * Otherwise, it may not be possible to receive any data through
 * the serial port using this class.
 *
 * :FIXME: Provide examples of the above potential problem.
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
     * The allowed set of baud rates.
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
    } ;

    enum CharacterSize {
        CHAR_SIZE_5  = CS5, //!< 5 bit characters.
        CHAR_SIZE_6  = CS6, //!< 6 bit characters.
        CHAR_SIZE_7  = CS7, //!< 7 bit characters.
        CHAR_SIZE_8  = CS8, //!< 8 bit characters.
        CHAR_SIZE_DEFAULT = CHAR_SIZE_8
    } ;

    enum StopBits {
        STOP_BITS_1,   //! 1 stop bit.
        STOP_BITS_2,   //! 2 stop bits.
        STOP_BITS_DEFAULT = STOP_BITS_1
    } ;

    enum Parity {
        PARITY_EVEN,     //!< Even parity.
        PARITY_ODD,      //!< Odd parity.
        PARITY_NONE,     //!< No parity i.e. parity checking disabled.
        PARITY_DEFAULT = PARITY_NONE
    } ;

    enum FlowControl {
        FLOW_CONTROL_HARD,
        FLOW_CONTROL_SOFT,
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
    } ;

    class NotOpen : public std::logic_error
    {
    public:
        NotOpen(const std::string& whatArg) :
            logic_error(whatArg) { }
    } ;

    class OpenFailed : public std::runtime_error
    {
    public:
        OpenFailed(const std::string& whatArg) :
            runtime_error(whatArg) { }
    } ;

    class AlreadyOpen : public std::logic_error
    {
    public:
        AlreadyOpen( const std::string& whatArg ) :
            logic_error(whatArg) { }
    } ;

    class UnsupportedBaudRate : public std::runtime_error
    {
    public:
        UnsupportedBaudRate( const std::string& whatArg ) :
            runtime_error(whatArg) { }
    } ;

    class ReadTimeout : public std::runtime_error
    {
    public:
        ReadTimeout() : runtime_error( "Read timeout" ) { }
    } ;

    /**
     * Constructor for a serial port.
     */
    explicit SerialPort( const std::string& serialPortName ) ;

    /**
     * Destructor.
     */
    virtual ~SerialPort() throw() ;

    /**
     * Open the serial port with the specified settings. A serial port
     * cannot be used till it is open.
     *
     * @throw AlreadyOpen This exception is thrown if the serial port
     * is already open.
     *
     * @throw OpenFailed This exception is thrown if the serial port
     * could not be opened.
     *
     * @throw std::invalid_argument This exception is thrown if an
     * invalid parameter value is specified.
     */
    void
    Open( const BaudRate      baudRate    = BAUD_DEFAULT,
          const CharacterSize charSize    = CHAR_SIZE_DEFAULT,
          const Parity        parityType  = PARITY_DEFAULT,
          const StopBits      stopBits    = STOP_BITS_DEFAULT,
          const FlowControl   flowControl = FLOW_CONTROL_DEFAULT )
        throw( AlreadyOpen,
               OpenFailed,
               UnsupportedBaudRate,
               std::invalid_argument ) ;

    /**
     * Check if the serial port is open for I/O.
     */
    bool
    IsOpen() const ;

    /**
     * Close the serial port. All settings of the serial port will be
     * lost and no more I/O can be performed on the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    void
    Close()
        throw(NotOpen) ;

    /**
     * Set the baud rate for the serial port to the specified value
     * (baudRate).
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     * @throw std::invalid_argument Thrown if an invalid baud rate is
     * specified.
     */
    void
    SetBaudRate( const BaudRate baudRate )
        throw( UnsupportedBaudRate,
               NotOpen,
               std::invalid_argument ) ;

    /**
     * Get the current baud rate for the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     */
    BaudRate
    GetBaudRate() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Set the character size for the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     * @throw std::invalid_argument Thrown if an invalid character
     * size is specified.
     */
    void
    SetCharSize( const CharacterSize charSize )
        throw( NotOpen,
               std::invalid_argument ) ;
    /**
     * Get the current character size for the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    CharacterSize
    GetCharSize() const
        throw(NotOpen) ;

    /**
     * Set the parity type for the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     * @throw std::invalid_argument Thrown if an invalid parity is
     * specified.
     */
    void
    SetParity( const Parity parityType )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * Get the parity type for the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    Parity
    GetParity() const
        throw(NotOpen) ;

    /**
     * Set the number of stop bits to be used with the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     * @throw std::invalid_argument Thrown if an invalid number of
     * stop bits is specified.
     */
    void
    SetNumOfStopBits( const StopBits numOfStopBits )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * Get the number of stop bits currently being used by the serial
     * port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    StopBits
    GetNumOfStopBits() const
        throw(NotOpen) ;

    /**
     * Set flow control.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     * @throw std::invalid_argument Thrown if an invalid flow control
     * is specified.
     */
    void
    SetFlowControl( const FlowControl   flowControl )
        throw( NotOpen,
               std::invalid_argument ) ;

    /**
     * Get the current flow control setting.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    FlowControl
    GetFlowControl() const
        throw( NotOpen ) ;

    /**
     * Check if data is available at the input of the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     *
     */
    bool
    IsDataAvailable() const
        throw(NotOpen) ;

    /**
     * Read a single byte from the serial port. If no data is
     * available in the specified number of milliseconds (msTimeout),
     * then this method will throw ReadTimeout exception. If msTimeout
     * is 0, then this method will block till data is available.
     */
    unsigned char
    ReadByte( const unsigned int msTimeout = 0 )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;

    /**
     * Read the specified number of bytes from the serial port. The
     * method will timeout if no data is received in the specified
     * number of milliseconds (msTimeout). If msTimeout is 0, then
     * this method will block till all requested bytes are
     * received. If numOfBytes is zero, then this method will keep
     * reading data till no more data is available at the serial
     * port. In all cases, all read data is available in dataBuffer on
     * return from this method.
     */
    typedef std::vector<unsigned char> DataBuffer ;
    void
    Read( DataBuffer&        dataBuffer,
          const unsigned int numOfBytes = 0,
          const unsigned int msTimeout  = 0 )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;


    /**
     * Read a line of characters from the serial port.
     */
    const std::string
    ReadLine( const unsigned int msTimeout = 0,
              const char         lineTerminator = '\n' )
        throw( NotOpen,
               ReadTimeout,
               std::runtime_error ) ;

    /**
     * Send a single byte to the serial port.
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     */
    void
    WriteByte(const unsigned char dataByte)
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Write the data from the specified vector to the serial port.
     */
    void
    Write(const DataBuffer& dataBuffer)
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Write a string to the serial port.
     */
    void
    Write(const std::string& dataString)
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Set the DTR line to the specified value.
     */
    void
    SetDtr( const bool dtrState = true )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Get the status of the DTR line.
     */
    bool
    GetDtr() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Set the RTS line to the specified value.
     */
    void
    SetRts( const bool rtsState = true )
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Get the status of the RTS line.
     */
    bool
    GetRts() const
        throw( NotOpen,
               std::runtime_error ) ;

    //void
    //SetCts( const bool ctsState = true )
    //    throw( NotOpen,
    //           std::runtime_error ) ;

    bool
    GetCts() const
        throw( NotOpen,
               std::runtime_error ) ;

    //void
    //SetDsr( const bool dsrState = true )
    //    throw( NotOpen,
    //           std::runtime_error ) ;

    bool
    GetDsr() const
        throw( NotOpen,
               std::runtime_error ) ;

    /**
     * Get the low-level file descriptor associated with the serial port. 
     *
     * \warning This is for advanced/power users only. Messing with this file
     * descriptor (such as closing it) will probably cause problems.  Be
     * careful when you use it. Using this file descriptor after the
     * controlling serial port has been closed or deleted (for example, when it
     * goes out of scope) will result in undefined behavior. You have been 
     * warned!
     *
     * @return The low-level file descriptor corresponding to the serial 
     * port.
     *
     * @throw This method will throw NotOpen if the serial port is not currently
     * open.
     */
    int GetFileDescriptor() const ;

private:
    /**
     * Prevent copying of objects of this class by declaring the copy
     * constructor private. This method is never defined.
     */
    SerialPort( const SerialPort& otherSerialPort ) ;

    /**
     * Prevent copying of objects of this class by declaring the assignment
     * operator private. This method is never defined.
     */
    SerialPort& operator=(const SerialPort& otherSerialPort ) ;

    /*
     * Forward declaration of the implementation class folowing the
     * PImpl idiom.
     */
    class SerialPortImpl ;

    /**
     * Pointer to implementation class instance.
     */
    SerialPortImpl* mSerialPortImpl ;
} ;

#endif // #ifndef _SerialPort_h_

