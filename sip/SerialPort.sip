
class SerialPort /NoDefaultCtors/
{
%TypeHeaderCode
#include <SerialPort.h>
%End
public:
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
        BAUD_DEFAULT = BAUD_57600
    } ;

    enum CharacterSize
    {
        CHAR_SIZE_5  = CS5, //!< 5 bit characters.
        CHAR_SIZE_6  = CS6, //!< 6 bit characters.
        CHAR_SIZE_7  = CS7, //!< 7 bit characters.
        CHAR_SIZE_8  = CS8, //!< 8 bit characters.
        CHAR_SIZE_DEFAULT = CHAR_SIZE_8
    } ;

    enum FlowControl
    {
        FLOW_CONTROL_HARD,
        // FLOW_CONTROL_SOFT,
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
    } ;

    enum Parity
    {
        PARITY_EVEN,     //!< Even parity.
        PARITY_ODD,      //!< Odd parity.
        PARITY_NONE,     //!< No parity i.e. parity checking disabled.
        PARITY_DEFAULT = PARITY_NONE
    } ;

    enum StopBits
    {
        STOP_BITS_1,   //! 1 stop bit.
        STOP_BITS_2,   //! 2 stop bits.
        STOP_BITS_DEFAULT = STOP_BITS_1
    } ;

    class NotOpen : std::logic_error
    {
    public:
        NotOpen(const std::string& whatArg) ;
        virtual ~NotOpen() throw() ;
%MethodCode
	        return ;
%End
    } ;

    class AlreadyOpen : std::logic_error
    {
    public:
        AlreadyOpen( const std::string& whatArg ) ;
        virtual ~AlreadyOpen() throw() ;
%MethodCode
            return ;
%End
    } ;

    class OpenFailed : std::runtime_error
    {
    public:
        OpenFailed( const std::string& whatArg ) ;
        virtual ~OpenFailed() throw() ;
%MethodCode
	        return ;
%End
    } ;

    class ReadTimeout : std::runtime_error
    {
    public:
        ReadTimeout( const std::string& whatArg ) ;
        virtual ~ReadTimeout() throw() ;
%MethodCode
            return ;
%End
    } ;

    explicit SerialPort( const std::string& serialPortName ) ;

    ~SerialPort() throw() ;

    void
    Open( const BaudRate      baudRate    = BAUD_DEFAULT,
          const CharacterSize charSize    = CHAR_SIZE_DEFAULT,
          const Parity        parityType  = PARITY_DEFAULT,
          const StopBits      stopBits    = STOP_BITS_DEFAULT,
          const FlowControl   flowControl = FLOW_CONTROL_DEFAULT )
        throw( SerialPort::AlreadyOpen,
               SerialPort::OpenFailed,
               std::invalid_argument ) ;

    void
    Close()
        throw( SerialPort::NotOpen ) ;

    void
    FlushInputBuffer()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    FlushOutputBuffer()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    
    void
    FlushIOBuffers()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    
    bool
    IsDataAvailable()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    bool
    IsOpen()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    SetDefaultSerialPortParameters()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    SetBaudRate( const BaudRate baudRate )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    BaudRate
    GetBaudRate() const
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    SetCharacterSize( const CharacterSize characterSize )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;
    
    CharacterSize
    GetCharacterSize() const
        throw(SerialPort::NotOpen) ;

    void
    SetFlowControl( const FlowControl flowControlType )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    FlowControl
    GetFlowControl() const
        throw( SerialPort::NotOpen ) ;

    void
    SetParity( const Parity parityType )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    Parity
    GetParity() const
        throw(SerialPort::NotOpen) ;

    void
    SetNumberOfStopBits( const StopBits numOfStopBits )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    StopBits
    GetNumberOfStopBits() const
        throw(SerialPort::NotOpen) ;

    void
    SetVMin( const short vmin )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    short
    GetVMin()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    SetVTime( const short vtime )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    short
    GetVTime()
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;   

    void
    SetDTR( const bool dtrState = true )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    bool
    GetDTR() 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    void
    SetRTS( const bool rtsState = true )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    bool
    GetRTS() 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    bool
    GetCTS() 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    bool
    GetDSR() 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    int
    GetFileDescriptor() 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    Read( unsigned char&     charBuffer,
          const unsigned int numberOfBytes = 0,
          const unsigned int msTimeout  = 0 )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;

    typedef std::vector<unsigned char> DataBuffer ;
    void
    Read( SerialPort::DataBuffer& dataBuffer,
          const unsigned int numOfBytes = 0,
          const unsigned int msTimeout  = 0 )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;

    void
    Read( std::string&       dataString,
          const unsigned int numberOfBytes = 0,
          const unsigned int msTimeout  = 0 )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;

    unsigned char
    ReadByte( unsigned char&     charBuffer,
              const unsigned int msTimeout = 0 )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;

    void
    ReadLine( std::string&       dataString,
              const char         lineTerminator = "\n",
              const unsigned int msTimeout = 0 )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;

    void
    Write( const unsigned char* charBuffer,
           const unsigned int   numberOfBytes )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    
    void
    Write( const SerialPort::DataBuffer& dataBuffer)
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    Write(const std::string& dataString)
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    WriteByte(const unsigned char charbuffer)
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

private:
    SerialPort( const SerialPort& otherSerialPort ) ;
} ;
