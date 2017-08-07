class SerialPort /NoDefaultCtors/
{
%TypeHeaderCode
#include <SerialPort.h>
#include <SerialPortConstants.h>
%End
public:
    explicit SerialPort();

    explicit SerialPort(const std::string&   fileName,
                        const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                        const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                        const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                        const Parity&        parityType      = Parity::PARITY_DEFAULT,
                        const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT);

    virtual ~SerialPort();

    void
    Open(const std::string& fileName,
         std::ios_base::openmode openMode = std::ios_base::in | std::ios_base::out);

    void
    Close();

    void
    FlushInputBuffer();

    void
    FlushOutputBuffer();
    
    void
    FlushIOBuffers();
    
    bool
    IsDataAvailable();

    bool
    IsOpen();

    void
    SetDefaultSerialPortParameters();

    void
    SetBaudRate(const BaudRate baudRate);

    BaudRate
    GetBaudRate();

    void
    SetCharacterSize(const CharacterSize characterSize);
    
    CharacterSize
    GetCharacterSize();

    void
    SetFlowControl(const FlowControl flowControlType);

    FlowControl
    GetFlowControl();

    void
    SetParity(const Parity parityType);

    Parity
    GetParity();

    void
    SetNumberOfStopBits(const StopBits numOfStopBits);

    StopBits
    GetNumberOfStopBits();

    void
    SetVMin(const short vmin);

    short
    GetVMin();

    void
    SetVTime(const short vtime);

    short
    GetVTime();

    void
    SetDTR(const bool dtrState = true);
    
    bool
    GetDTR();
    
    void
    SetRTS(const bool rtsState = true);

    bool
    GetRTS();

    bool
    GetCTS();

    bool
    GetDSR();

    int
    GetFileDescriptor();

    void
    Read(unsigned char&     charBuffer,
         const unsigned int numberOfBytes = 0,
         const unsigned int msTimeout  = 0);

    typedef std::vector<unsigned char> DataBuffer;
    
    void
    Read(DataBuffer& dataBuffer,
         const unsigned int numOfBytes = 0,
         const unsigned int msTimeout  = 0);

    void
    Read(std::string&       dataString,
         const unsigned int numberOfBytes = 0,
         const unsigned int msTimeout  = 0);

    unsigned char
    ReadByte(unsigned char&     charBuffer,
             const unsigned int msTimeout = 0);

    void
    ReadLine(std::string&       dataString,
             const char         lineTerminator = "\n",
             const unsigned int msTimeout = 0);

    void
    Write(const unsigned char* charBuffer,
          const unsigned int   numberOfBytes);
    
    void
    Write(const DataBuffer& dataBuffer);

    void
    Write(const std::string& dataString);

    void
    WriteByte(const unsigned char charbuffer);

private:
    SerialPort(const SerialPort& otherSerialPort);
} ;