#ifndef _SerialPort_h_
#define _SerialPort_h_

#ifndef _std_string_INCLUDED_
#    include <string>
#    define _std_string_INCLUDED_
#endif

#ifndef _std_stdexcept_INCLUDED_
#    include <stdexcept>
#    define _std_stdexcept_INCLUDED_
#endif

#ifndef _termios_h_INCLUDED_
#    include <termios.h>
#    define _termios_h_INCLUDED_
#endif

class SerialPortImpl ;

class SerialPort {
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
        BAUD_460800  = B460800,
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
        // FLOW_CONTROL_SOFT,
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
    } ;

    class NotOpen : public std::logic_error {
    public:
        NotOpen(const std::string& whatArg) :
            logic_error(whatArg) { }
    } ;
    
    class OpenFailed : public std::runtime_error {
    public:
        OpenFailed(const std::string& whatArg) :
            runtime_error(whatArg) { }
    } ;

    class AlreadyOpen : public std::logic_error {
    public:
        AlreadyOpen( const std::string& whatArg ) :
            logic_error(whatArg) { }
    } ;

    class UnsupportedBaudRate : public std::runtime_error {
    public:
        UnsupportedBaudRate( const std::string& whatArg ) :
            runtime_error(whatArg) { }
    } ;

    class ReadTimeout : public std::runtime_error {
    public:
        ReadTimeout() : runtime_error( "Read timeout" ) { }
    } ;

    /**
     * Constructor for a serial port. 
     */
    SerialPort( const std::string& serialPortName ) ;

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
     * Send a single byte to the serial port. 
     *
     * @throw NotOpen Thrown if this method is called while the serial
     * port is not open.
     */
    void 
    WriteByte(const unsigned char dataByte) 
        throw( NotOpen, 
               std::runtime_error ) ;
private:
    SerialPort( const SerialPort& otherSerialPort ) ;
    SerialPort& operator=(const SerialPort& otherSerialPort ) ;
    SerialPortImpl* mSerialPortImpl ;
} ;

#endif // #ifndef _SerialPort_h_

