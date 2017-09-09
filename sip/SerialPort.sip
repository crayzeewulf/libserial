class SerialPort
{

%TypeHeaderCode
#include <SerialPort.h>
using namespace LibSerial;
%End

public:
    explicit SerialPort();

    explicit SerialPort(const std::string&              fileName,
                        const LibSerial::BaudRate&      baudRate        = LibSerial::BaudRate::BAUD_DEFAULT,
                        const LibSerial::CharacterSize& characterSize   = LibSerial::CharacterSize::CHAR_SIZE_DEFAULT,
                        const LibSerial::FlowControl&   flowControlType = LibSerial::FlowControl::FLOW_CONTROL_DEFAULT,
                        const LibSerial::Parity&        parityType      = LibSerial::Parity::PARITY_DEFAULT,
                        const LibSerial::StopBits&      stopBits        = LibSerial::StopBits::STOP_BITS_DEFAULT);

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
    SetBaudRate(const LibSerial::BaudRate baudRate);

    LibSerial::BaudRate
    GetBaudRate();

    void
    SetCharacterSize(const LibSerial::CharacterSize characterSize);
    
    LibSerial::CharacterSize
    GetCharacterSize();

    void
    SetFlowControl(const LibSerial::FlowControl flowControlType);

    LibSerial::FlowControl
    GetFlowControl();

    void
    SetParity(const LibSerial::Parity parityType);

    LibSerial::Parity
    GetParity();

    void
    SetNumberOfStopBits(const LibSerial::StopBits numOfStopBits);

    LibSerial::StopBits
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

    typedef std::vector<unsigned char> DataBuffer;
    
    // NOTE: Python3 provides a mechanism for method overloading, however Python2 does not.
    // void
    // Read(char&              charBuffer,
    //      const unsigned int numberOfBytes = 0,
    //      const unsigned int msTimeout  = 0);

    void
    Read(unsigned char&     charBuffer,
         const unsigned int numberOfBytes = 0,
         const unsigned int msTimeout  = 0);

    void
    Read(DataBuffer& dataBuffer,
         const unsigned int numOfBytes = 0,
         const unsigned int msTimeout  = 0);

    void
    Read(std::string&       dataString,
         const unsigned int numberOfBytes = 0,
         const unsigned int msTimeout  = 0);

    void
    ReadByte(unsigned char&     charBuffer,
             const unsigned int msTimeout = 0);

    // NOTE: Python3 provides a mechanism for method overloading, however Python2 does not.
    // void
    // ReadByte(char&              charBuffer,
    //          const unsigned int msTimeout = 0);

    void
    ReadLine(std::string&       dataString,
             const char         lineTerminator = 10, // Sip does not like '\n'
             const unsigned int msTimeout = 0);

    // NOTE: Python3 provides a mechanism for method overloading, however Python2 does not.
    // void
    // Write(const char*        charBuffer,
    //       const unsigned int numberOfBytes);
    
    void
    Write(const unsigned char* charBuffer,
          const unsigned int   numberOfBytes);
    
    void
    Write(const DataBuffer& dataBuffer);

    void
    Write(const std::string& dataString);

    // NOTE: Python3 provides a mechanism for method overloading, however Python2 does not.
    // void
    // WriteByte(const char charbuffer);

    void
    WriteByte(const unsigned char charbuffer);

private:
    SerialPort(const SerialPort& otherSerialPort);
} ;
