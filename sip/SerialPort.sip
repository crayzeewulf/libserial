
class SerialPort /NoDefaultCtors/
{
%TypeHeaderCode
#include <SerialPort.h>
%End
public:
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

    class NotOpen : std::logic_error
    {
    public:
        NotOpen(const std::string& whatArg) ;
        virtual ~NotOpen() throw() ;
%MethodCode
	        return ;
%End
    } ;

    class OpenFailed : std::runtime_error
    {
    public:
        OpenFailed(const std::string& whatArg) ;
        virtual ~OpenFailed() throw() ;
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

    class UnsupportedBaudRate : std::runtime_error
    {
    public:
        UnsupportedBaudRate( const std::string& whatArg ) ;
        virtual ~UnsupportedBaudRate() throw() ;
%MethodCode
	        return ;
%End
    } ;

    class ReadTimeout : std::runtime_error
    {
    public:
        ReadTimeout() ;
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
               SerialPort::UnsupportedBaudRate,
               std::invalid_argument ) ;

    bool
    IsOpen() const ;

    void
    Close()
        throw(SerialPort::NotOpen) ;

    void
    SetBaudRate( const BaudRate baudRate )
        throw( SerialPort::UnsupportedBaudRate,
               SerialPort::NotOpen,
               std::invalid_argument ) ;

    BaudRate
    GetBaudRate() const
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    void
    SetCharSize( const CharacterSize charSize )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;
    CharacterSize
    GetCharSize() const
        throw(SerialPort::NotOpen) ;

    void
    SetParity( const Parity parityType )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    Parity
    GetParity() const
        throw(SerialPort::NotOpen) ;

    void
    SetNumOfStopBits( const StopBits numOfStopBits )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    StopBits
    GetNumOfStopBits() const
        throw(SerialPort::NotOpen) ;

    void
    SetFlowControl( const FlowControl   flowControl )
        throw( SerialPort::NotOpen,
               std::invalid_argument ) ;

    FlowControl
    GetFlowControl() const
        throw( SerialPort::NotOpen ) ;

    bool
    IsDataAvailable() const
        throw(SerialPort::NotOpen) ;

    unsigned char
    ReadByte( const unsigned int msTimeout = 0 )
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

    const std::string
    ReadLine( const unsigned int msTimeout,
              const char         lineTerminator )
        throw( SerialPort::NotOpen,
               SerialPort::ReadTimeout,
               std::runtime_error ) ;
    void
    WriteByte(const unsigned char dataByte)
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
    SetDtr( const bool dtrState = true )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    bool
    GetDtr() const 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
    void
    SetRts( const bool rtsState = true )
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    bool
    GetRts() const 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    //void
    //SetCts( const bool ctsState = true )
    //    throw( SerialPort::NotOpen,
    //           std::runtime_error ) ;
    
    bool
    GetCts() const 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;

    //void
    //SetDsr( const bool dsrState = true )
    //    throw( SerialPort::NotOpen,
    //           std::runtime_error ) ;
    
    bool
    GetDsr() const 
        throw( SerialPort::NotOpen,
               std::runtime_error ) ;
private:
    SerialPort( const SerialPort& otherSerialPort ) ;
} ;

