/**
 * @file LibSerialTest.cpp
 * @copyright LibSerial
 */

#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <SerialPort.h>
#include <SerialStream.h>

// Default Serial Ports.
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

using namespace LibSerial;

class LibSerialTest
    : public ::testing::Test
{
public:

protected:

    BaudRate        baudRates[25];
    CharacterSize   characterSizes[4];
    FlowControl     flowControlTypes[3];
    Parity          parityTypes[3];
    StopBits        stopBits[2];

    SerialStream serialStream1;
    SerialStream serialStream2;

    SerialPort serialPort1;
    SerialPort serialPort2;

    std::string readString1  = " ";
    std::string writeString1 = " ";

    std::string readString2  = " ";
    std::string writeString2 = " ";

    char writeByte = 'a';
    char readByte  = 'b';

    virtual void SetUp()
    {
        writeString1 = "Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)";
        writeString2 = "The secret of the man who is universally interesting is that he is universally interested. - William Dean Howells";

        baudRates[0]   =  BaudRate::BAUD_50;
        baudRates[1]   =  BaudRate::BAUD_75;
        baudRates[2]   =  BaudRate::BAUD_110;
        baudRates[3]   =  BaudRate::BAUD_134;
        baudRates[4]   =  BaudRate::BAUD_150;
        baudRates[5]   =  BaudRate::BAUD_200;
        baudRates[6]   =  BaudRate::BAUD_300;
        baudRates[7]   =  BaudRate::BAUD_600;
        baudRates[8]   =  BaudRate::BAUD_1200;
        baudRates[9]   =  BaudRate::BAUD_1800;
        baudRates[10]  =  BaudRate::BAUD_2400;
        baudRates[11]  =  BaudRate::BAUD_4800;
        baudRates[12]  =  BaudRate::BAUD_9600;
        baudRates[13]  =  BaudRate::BAUD_19200;
        baudRates[14]  =  BaudRate::BAUD_38400;
        baudRates[15]  =  BaudRate::BAUD_57600;
        baudRates[16]  =  BaudRate::BAUD_115200;
        baudRates[17]  =  BaudRate::BAUD_230400;
        baudRates[18]  =  BaudRate::BAUD_460800;
        baudRates[19]  =  BaudRate::BAUD_921600;
        baudRates[20]  =  BaudRate::BAUD_1000000;
        baudRates[21]  =  BaudRate::BAUD_1500000;
        baudRates[22]  =  BaudRate::BAUD_2000000;
        baudRates[23]  =  BaudRate::BAUD_2500000;
        baudRates[24]  =  BaudRate::BAUD_3000000;

        characterSizes[0] = CharacterSize::CHAR_SIZE_5;
        characterSizes[1] = CharacterSize::CHAR_SIZE_6;
        characterSizes[2] = CharacterSize::CHAR_SIZE_7;
        characterSizes[3] = CharacterSize::CHAR_SIZE_8;

        flowControlTypes[0] = FlowControl::FLOW_CONTROL_NONE;
        flowControlTypes[1] = FlowControl::FLOW_CONTROL_HARDWARE;
        flowControlTypes[2] = FlowControl::FLOW_CONTROL_SOFTWARE;

        parityTypes[0] = Parity::PARITY_EVEN;
        parityTypes[1] = Parity::PARITY_ODD;
        parityTypes[2] = Parity::PARITY_NONE;

        stopBits[0] = StopBits::STOP_BITS_1;
        stopBits[1] = StopBits::STOP_BITS_2;
    }

    long unsigned int getTimeInMicroSeconds()
    {
        std::chrono::high_resolution_clock::duration timeNow = 
            std::chrono::high_resolution_clock::now().time_since_epoch();

        return std::chrono::duration_cast<std::chrono::microseconds>(timeNow).count();
    }
    
    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream1.SetBaudRate(baudRates[i]);
            BaudRate baudRate = serialStream1.GetBaudRate();
            ASSERT_EQ(baudRate, baudRates[i]);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetCharacterSize()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        // @NOTE - Smaller Character Size values do not work in Linux.
        for (size_t i = 2; i < 4; i++)
        {
            serialStream1.SetCharacterSize(characterSizes[i]);
            CharacterSize characterSize = serialStream1.GetCharacterSize();
            ASSERT_EQ(characterSize, characterSizes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetFlowControl(flowControlTypes[i]);
            FlowControl flowControl = serialStream1.GetFlowControl();
            ASSERT_EQ(flowControl, flowControlTypes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetParity(parityTypes[i]);
            Parity parity = serialStream1.GetParity();
            ASSERT_EQ(parity, parityTypes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        StopBits numberOfStopBits;

        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialStream1.SetNumberOfStopBits(stopBits[0]);
        numberOfStopBits = serialStream1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[0]);

        serialStream1.SetNumberOfStopBits(stopBits[1]);
        numberOfStopBits = serialStream1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[1]);

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetVMin()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVMin(i);
            short vMin = serialStream1.GetVMin();
            ASSERT_EQ(vMin, i);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetVTime()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVTime(i);
            short vTime = serialStream1.GetVTime();
            ASSERT_EQ(vTime, i);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamReadWriteByte()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        writeByte = 'a';
        serialStream1.write(&writeByte, 1);
        serialStream2.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        writeByte = 'A';
        serialStream1 << writeByte;
        serialStream2.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamGetLineWriteString()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream1 << writeString1 << std::endl;
        getline(serialStream2, readString1);
        ASSERT_EQ(readString1, writeString1);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamGetWriteByte()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        writeByte = 'a';
        serialStream1.write(&writeByte, 1);
        serialStream2.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        writeByte = 'A';
        serialStream1 << writeByte;
        serialStream2.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }


    //----------------------- Serial Port Unit Tests ------------------------//

    void testSerialPortOpenClose()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortFlushInputBuffer()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.FlushInputBuffer();
        serialPort2.FlushInputBuffer();
        
        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        writeByte = 'A';
        serialPort1.WriteByte(writeByte);
        serialPort2.WriteByte(writeByte);

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());

        usleep(25000);

        ASSERT_TRUE(serialPort1.IsDataAvailable());
        ASSERT_TRUE(serialPort2.IsDataAvailable());

        serialPort1.FlushInputBuffer();
        serialPort2.FlushInputBuffer();

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortFlushOutputBuffer()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.FlushInputBuffer();
        serialPort2.FlushInputBuffer();
        
        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        writeByte = 'A';
        serialPort1.WriteByte(writeByte);
        serialPort1.FlushOutputBuffer();
        
        serialPort2.WriteByte(writeByte);
        serialPort2.FlushOutputBuffer();

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortFlushIOBuffers()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        writeByte = 'A';

        serialPort1.WriteByte(writeByte);
        serialPort1.FlushIOBuffers();

        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort2.WriteByte(writeByte);
        serialPort2.FlushIOBuffers();

        ASSERT_FALSE(serialPort1.IsDataAvailable());

        serialPort1.WriteByte(writeByte);
        serialPort2.WriteByte(writeByte);

        usleep(25000);

        ASSERT_TRUE(serialPort1.IsDataAvailable());
        ASSERT_TRUE(serialPort2.IsDataAvailable());

        serialPort1.FlushIOBuffers();
        serialPort2.FlushIOBuffers();

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortIsDataAvailableTest()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        writeByte = 'A';

        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());
        
        usleep(25000);
        ASSERT_TRUE(serialPort2.IsDataAvailable());

        unsigned char readByte;

        serialPort2.ReadByte(readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
                
        usleep(25000);
        ASSERT_TRUE(serialPort1.IsDataAvailable());
        
        serialPort1.ReadByte(readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetBaudRate()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialPort1.SetBaudRate(baudRates[i]);
            BaudRate baudRate = serialPort1.GetBaudRate();
            ASSERT_EQ(baudRate, baudRates[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetCharacterSize()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        // @NOTE - Smaller CharSize values appear to be invalid on x86 Linux.
        for (size_t i = 2; i < 4; i++)
        {
            serialPort1.SetCharacterSize(characterSizes[i]);
            CharacterSize characterSize = serialPort1.GetCharacterSize();
            ASSERT_EQ(characterSize, characterSizes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetFlowControl()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        // @NOTE - FLOW_CONTROL_SOFT flow control appears to be invalid on x86 Linux.
        for (size_t i = 0; i < 2; i++)
        {
            serialPort1.SetFlowControl(flowControlTypes[i]);
            FlowControl flowControl = serialPort1.GetFlowControl();
            ASSERT_EQ(flowControl, flowControlTypes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetParity()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialPort1.SetParity(parityTypes[i]);
            Parity parity = serialPort1.GetParity();
            ASSERT_EQ(parity, parityTypes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetStopBits()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());
        
        StopBits numberOfStopBits;

        serialPort1.SetNumberOfStopBits(stopBits[0]);
        numberOfStopBits = serialPort1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[0]);

        serialPort1.SetNumberOfStopBits(stopBits[1]);
        numberOfStopBits = serialPort1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[1]);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetVMin()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialPort1.SetVMin(i);
            short vMin = serialPort1.GetVMin();
            ASSERT_EQ(vMin, i);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetVTime()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialPort1.SetVTime(i);
            short vTime = serialPort1.GetVTime();
            ASSERT_EQ(vTime, i);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetDTR()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetDTR(true);
        bool dtrLine = serialPort1.GetDTR();
        ASSERT_TRUE(dtrLine);

        serialPort1.SetDTR(false);
        dtrLine = serialPort1.GetDTR();
        ASSERT_FALSE(dtrLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetRTS(true);
        bool rtsLine = serialPort1.GetRTS();
        ASSERT_TRUE(rtsLine);

        serialPort1.SetRTS(false);
        rtsLine = serialPort1.GetRTS();
        ASSERT_FALSE(rtsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetCTS()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        bool ctsLine = serialPort1.GetCTS();
        ASSERT_FALSE(ctsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetDSR()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        bool dsrStatus = serialPort1.GetDSR();
        ASSERT_FALSE(dsrStatus);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetFileDescriptor()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        int fileDescriptor = serialPort1.GetFileDescriptor();
        ASSERT_GE(fileDescriptor, 0);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortReadCharBufferWriteCharBuffer(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
        
        unsigned char readByte;

        serialPort1.Read(readByte, 1, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

        serialPort2.Read(readByte, 1, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadDataBufferWriteDataBuffer(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        SerialPort::DataBuffer readDataBuffer;
        SerialPort::DataBuffer writeDataBuffer;

        // Test using ASCII characters.
        for (size_t i = 48; i < 122; i++)
        {
            writeDataBuffer.push_back(i);
        }

        serialPort1.Write(writeDataBuffer);
        tcdrain(serialPort1.GetFileDescriptor());
        
        serialPort2.Read(readDataBuffer, 74, timeOutMilliseconds);

        ASSERT_EQ(readDataBuffer, writeDataBuffer);
        ASSERT_EQ(readDataBuffer.size(), writeDataBuffer.size());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadStringWriteString(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.Write(writeString1);
        tcdrain(serialPort1.GetFileDescriptor());

        serialPort2.Write(writeString2);
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.Read(readString2, writeString2.size(), timeOutMilliseconds);
        ASSERT_EQ(readString2, writeString2);
        ASSERT_EQ(readString2.size(), writeString2.size());
        
        serialPort2.Read(readString1, writeString1.size(), timeOutMilliseconds);
        ASSERT_EQ(readString1, writeString1);
        ASSERT_EQ(readString1.size(), writeString1.size());

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadByteWriteByte(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());

        unsigned char readByte;

        serialPort1.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

        serialPort2.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadLineWriteString(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.Write(writeString1 + '\n');
        tcdrain(serialPort1.GetFileDescriptor());

        serialPort2.Write(writeString2 + '\n');
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString2, writeString2 + '\n');
        ASSERT_EQ(readString2.size(), writeString2.size() + 1);
        
        serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString1, writeString1 + '\n');
        ASSERT_EQ(readString1.size(), writeString1.size() + 1);
       
        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialStreamToSerialPortReadWrite(const int timeOutMilliseconds)
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        serialStream1.Open(TEST_SERIAL_PORT_2);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialPort1.SetBaudRate(baudRates[16]);
        serialStream1.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate1 = serialPort1.GetBaudRate();
        BaudRate baudRate2 = serialStream1.GetBaudRate();
        
        ASSERT_EQ(baudRate1, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

        serialStream1 << writeString1 << std::endl;
        serialPort1.ReadLine(readString1, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString1, writeString1 + '\n');
        ASSERT_EQ(readString1.size(), writeString1.size() + 1);
       
        serialPort1.Write(writeString2 + '\n');
        tcdrain(serialPort1.GetFileDescriptor());
        getline(serialStream1, readString2);
        ASSERT_EQ(readString2, writeString2);

        serialPort1.Close();
        serialStream1.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void serialStream1ThreadLoop()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream1.SetBaudRate(BaudRate::BAUD_115200);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;

        loopStartTimeMicroseconds = getTimeInMicroSeconds();

        while (timeElapsedMicroSeconds < 30000000)
        {
            serialStream1 << writeString1 << std::endl;
            getline(serialStream1, readString2);
            ASSERT_EQ(readString2, writeString2);

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
        }

        serialStream1.Close();
        return;
    }

    void serialStream2ThreadLoop()
    {
        serialStream2.Open(TEST_SERIAL_PORT_2);
        serialStream2.SetBaudRate(BaudRate::BAUD_115200);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;

        loopStartTimeMicroseconds = getTimeInMicroSeconds();

        while (timeElapsedMicroSeconds < 30000000)
        {
            serialStream2 << writeString2 << std::endl;
            getline(serialStream2, readString1);
            ASSERT_EQ(readString1, writeString1);

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
        }

        serialStream2.Close();
        return;
    }

    void serialPort1ThreadLoop()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort1.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        long unsigned int loopStartTimeMicroseconds = getTimeInMicroSeconds();
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeOutMilliseconds = 75;

        loopStartTimeMicroseconds = getTimeInMicroSeconds();

        while (timeElapsedMicroSeconds < 30000000)
        {
            serialPort1.Write(writeString1 + '\n');
            tcdrain(serialPort1.GetFileDescriptor());

            serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
            ASSERT_EQ(readString2, writeString2 + '\n');

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
        }

        serialPort1.Close();
        return;
    }

    void serialPort2ThreadLoop()
    {
        serialPort2.Open(TEST_SERIAL_PORT_2);
        serialPort2.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        long unsigned int loopStartTimeMicroseconds = getTimeInMicroSeconds();
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeOutMilliseconds = 75;

        loopStartTimeMicroseconds = getTimeInMicroSeconds();

        while (timeElapsedMicroSeconds < 30000000)
        {
            serialPort2.Write(writeString2 + '\n');
            tcdrain(serialPort2.GetFileDescriptor());

            serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);
            ASSERT_EQ(readString1, writeString1 + '\n');

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
        }

        serialPort2.Close();
        return;
    }

    void testMultiThreadSerialStreamReadWrite()
    {
        std::thread serialStream1Thread(serialStream1ThreadLoop, this);
        std::thread serialStream2Thread(serialStream2ThreadLoop, this);

        serialStream1Thread.join();
        serialStream2Thread.join();
    }

    void testMultiThreadSerialPortReadWrite()
    {
        std::thread serialPort1Thread(serialPort1ThreadLoop, this);
        std::thread serialPort2Thread(serialPort2ThreadLoop, this);

        serialPort1Thread.join();
        serialPort2Thread.join();
    }
};


//------------------------ Serial Stream Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open() and Close() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadWriteByte)
{
    SCOPED_TRACE("Serial Stream ReadByte() and WriteByte() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamReadWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetLineWriteString)
{
    SCOPED_TRACE("Serial Stream GetLine() and Write(string) Test");
        
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamGetLineWriteString();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetWriteByte)
{
    SCOPED_TRACE("Serial Stream Get() and WriteByte() Test");
        
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamGetWriteByte();
    }
}


//------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushInputBuffer)
{
    SCOPED_TRACE("Serial Port FlushInputBuffer() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortFlushInputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Port FlushOutputBuffer() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortFlushOutputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushIOBuffers)
{
    SCOPED_TRACE("Serial Port FlushIOBuffers() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortFlushIOBuffers();
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDTR() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetRTS() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetCTS)
{
    SCOPED_TRACE("Serial Port GetCTS() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetDSR)
{
    SCOPED_TRACE("Serial Port GetDSR() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortGetFileDescriptor)
{
    SCOPED_TRACE("Serial Port GetFileDescriptor() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetFileDescriptor();
    }
}

TEST_F(LibSerialTest, testSerialPortReadCharBufferWriteCharBuffer)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port Read(unsigned char) and Write(unsigned char) Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadCharBufferWriteCharBuffer(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortReadDataBufferWriteDataBuffer)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port Read(DataBuffer) and Write(DataBuffer) Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadDataBufferWriteDataBuffer(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortReadStringWriteString)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port Read(string) and Write(string) Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadStringWriteString(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortReadByteWriteByte)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadByteWriteByte(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortReadLineWriteString)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port ReadLine() and Write(string) Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadLineWriteString(timeOutMilliseconds);
    }
}


//----------------- Serial Stream to Serial Port Unit Test ------------------//

TEST_F(LibSerialTest, testSerialStreamToSerialPortReadWrite)
{
    int timeOutMilliseconds = 25;

    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamToSerialPortReadWrite(timeOutMilliseconds);
    }
}


//----------------------- Multiple Thread Unit Tests ------------------------//

TEST_F(LibSerialTest, testMultiThreadSerialStreamReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Stream Communication.");
    testMultiThreadSerialStreamReadWrite();
    sleep(1);
}

TEST_F(LibSerialTest, testMultiThreadSerialPortReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Port Communication.");
    testMultiThreadSerialPortReadWrite();
    sleep(1);
}
