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
    
    size_t numberOfTestIterations = 10;

protected:

    size_t timeOutMilliseconds = 30;
    size_t threadTimeOutMicroseconds = 5000000;

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

    size_t getTimeInMicroSeconds()
    {
        std::chrono::high_resolution_clock::duration timeNow = 
            std::chrono::high_resolution_clock::now().time_since_epoch();

        return std::chrono::duration_cast<std::chrono::microseconds>(timeNow).count();
    }
    
    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamFlushInputBuffer()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialStream1.FlushInputBuffer();
        serialStream2.FlushInputBuffer();
        
        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        char writeByte = 'A';

        serialStream1.write(&writeByte, 1);
        serialStream2.write(&writeByte, 1);

        tcdrain(serialStream1.GetFileDescriptor());
        tcdrain(serialStream2.GetFileDescriptor());

        usleep(25000);

        ASSERT_TRUE(serialStream1.IsDataAvailable());
        ASSERT_TRUE(serialStream2.IsDataAvailable());

        serialStream1.FlushInputBuffer();
        serialStream2.FlushInputBuffer();

        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        serialStream1.Close();
        serialStream2.Close();
        
        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamFlushOutputBuffer()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream1.FlushInputBuffer();
        serialStream2.FlushInputBuffer();
        
        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        char writeByte = 'A';
        
        serialStream1.write(&writeByte, 1);
        serialStream1.FlushOutputBuffer();
        
        serialStream2.write(&writeByte, 1);
        serialStream2.FlushOutputBuffer();

        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        serialStream1.Close();
        serialStream2.Close();
        
        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamFlushIOBuffers()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        char writeByte = 'a';

        serialStream1.write(&writeByte, 1);
        serialStream1.FlushIOBuffers();

        ASSERT_FALSE(serialStream2.IsDataAvailable());

        serialStream2.write(&writeByte, 1);
        serialStream2.FlushIOBuffers();

        ASSERT_FALSE(serialStream1.IsDataAvailable());

        serialStream1.write(&writeByte, 1);
        serialStream2.write(&writeByte, 1);

        usleep(25000);

        ASSERT_TRUE(serialStream1.IsDataAvailable());
        ASSERT_TRUE(serialStream2.IsDataAvailable());

        serialStream1.FlushIOBuffers();
        serialStream2.FlushIOBuffers();

        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        serialStream1.Close();
        serialStream2.Close();
        
        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamIsDataAvailableTest()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        char writeByte = 'a';
        char readByte = 'b';

        serialStream1.write(&writeByte, 1);
        tcdrain(serialStream1.GetFileDescriptor());
        
        usleep(25000);
        ASSERT_TRUE(serialStream2.IsDataAvailable());

        serialStream2.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream2.write(&writeByte, 1);
        tcdrain(serialStream2.GetFileDescriptor());
                
        usleep(25000);
        ASSERT_TRUE(serialStream1.IsDataAvailable());
        
        serialStream1.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        ASSERT_FALSE(serialStream1.IsDataAvailable());
        ASSERT_FALSE(serialStream2.IsDataAvailable());

        serialStream1.Close();
        serialStream2.Close();
        
        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream1.SetBaudRate(baudRates[i]);
            serialStream2.SetBaudRate(baudRates[i]);

            BaudRate baudRate1 = serialStream1.GetBaudRate();
            BaudRate baudRate2 = serialStream2.GetBaudRate();

            ASSERT_EQ(baudRate1, baudRates[i]);
            ASSERT_EQ(baudRate2, baudRates[i]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetCharacterSize()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        // @NOTE - Smaller Character Size values do not work in Linux.
        for (size_t i = 2; i < 4; i++)
        {
            serialStream1.SetCharacterSize(characterSizes[i]);
            serialStream2.SetCharacterSize(characterSizes[i]);

            CharacterSize characterSize1 = serialStream1.GetCharacterSize();
            CharacterSize characterSize2 = serialStream2.GetCharacterSize();

            ASSERT_EQ(characterSize1, characterSizes[i]);
            ASSERT_EQ(characterSize2, characterSizes[i]);

            usleep(10);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        FlowControl flowControl1 = FlowControl::FLOW_CONTROL_DEFAULT;
        FlowControl flowControl2 = FlowControl::FLOW_CONTROL_DEFAULT;

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetFlowControl(flowControlTypes[i]);
            serialStream1.SetFlowControl(flowControlTypes[i]);

            flowControl1 = serialStream1.GetFlowControl();
            flowControl2 = serialStream1.GetFlowControl();

            ASSERT_EQ(flowControl1, flowControlTypes[i]);
            ASSERT_EQ(flowControl2, flowControlTypes[i]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        Parity parity1 = Parity::PARITY_DEFAULT;
        Parity parity2 = Parity::PARITY_DEFAULT;

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetParity(parityTypes[i]);
            serialStream2.SetParity(parityTypes[i]);

            parity1 = serialStream1.GetParity();
            parity2 = serialStream2.GetParity();

            ASSERT_EQ(parity1, parityTypes[i]);
            ASSERT_EQ(parity2, parityTypes[i]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        StopBits numberOfStopBits1 = StopBits::STOP_BITS_DEFAULT;
        StopBits numberOfStopBits2 = StopBits::STOP_BITS_DEFAULT;

        for (size_t i = 0; i < 2; i++)
        {
            serialStream1.SetNumberOfStopBits(stopBits[0]);
            serialStream2.SetNumberOfStopBits(stopBits[0]);

            numberOfStopBits1 = serialStream1.GetNumberOfStopBits();
            numberOfStopBits2 = serialStream2.GetNumberOfStopBits();

            ASSERT_EQ(numberOfStopBits1, stopBits[0]);
            ASSERT_EQ(numberOfStopBits2, stopBits[0]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetVMin()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        short vMin1 = 0;
        short vMin2 = 0;

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVMin(i);
            serialStream2.SetVMin(i);

            vMin1 = serialStream1.GetVMin();
            vMin2 = serialStream2.GetVMin();

            ASSERT_EQ(vMin1, i);
            ASSERT_EQ(vMin2, i);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetVTime()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        short vTime1 = 0;
        short vTime2 = 0;

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVTime(i);
            serialStream2.SetVTime(i);

            vTime1 = serialStream1.GetVTime();
            vTime2 = serialStream2.GetVTime();

            ASSERT_EQ(vTime1, i);
            ASSERT_EQ(vTime2, i);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamReadWriteByte()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        char writeByte1 = 'a';
        char writeByte2 = 'A';

        char readByte1  = 'b';
        char readByte2  = 'B';

        serialStream1.write(&writeByte1, 1);
        serialStream2.write(&writeByte2, 1);

        serialStream1.read(&readByte2, 1);
        serialStream2.read(&readByte1, 1);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

        serialStream1 << writeByte1;
        serialStream2 << writeByte2;

        serialStream1.read(&readByte2, 1);
        serialStream2.read(&readByte1, 1);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

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
        serialStream2 << writeString2 << std::endl;

        getline(serialStream1, readString2);
        getline(serialStream2, readString1);

        ASSERT_EQ(readString1, writeString1);
        ASSERT_EQ(readString2, writeString2);

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

        char writeByte1 = 'a';
        char writeByte2 = 'A';

        char readByte1  = 'b';
        char readByte2  = 'B';

        serialStream1.write(&writeByte1, 1);
        serialStream2.write(&writeByte2, 1);

        serialStream1.get(readByte2);
        serialStream2.get(readByte1);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

        serialStream1 << writeByte1;
        serialStream2 << writeByte2;

        serialStream1.get(readByte2);
        serialStream2.get(readByte1);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

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

        char writeByte = 'A';

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

        char writeByte = 'A';
        
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

        char writeByte = 'a';

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

        char writeByte = 'a';
        unsigned char readByte = 'b';

        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());
        
        usleep(25000);
        ASSERT_TRUE(serialPort2.IsDataAvailable());

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
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool dtrLine1 = false;
        bool dtrLine2 = false;

        serialPort1.SetDTR(true);
        serialPort2.SetDTR(true);

        dtrLine1 = serialPort1.GetDTR();
        dtrLine2 = serialPort2.GetDTR();

        ASSERT_TRUE(dtrLine1);
        ASSERT_TRUE(dtrLine2);

        serialPort1.SetDTR(false);
        serialPort2.SetDTR(false);

        dtrLine1 = serialPort1.GetDTR();
        dtrLine2 = serialPort2.GetDTR();

        ASSERT_FALSE(dtrLine1);
        ASSERT_FALSE(dtrLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool rtsLine1 = false;
        bool rtsLine2 = false;

        serialPort1.SetRTS(true);
        serialPort2.SetRTS(true);
        
        rtsLine1 = serialPort1.GetRTS();
        rtsLine2 = serialPort2.GetRTS();
        
        ASSERT_TRUE(rtsLine1);
        ASSERT_TRUE(rtsLine2);

        serialPort1.SetRTS(false);
        serialPort2.SetRTS(false);
        
        rtsLine1 = serialPort1.GetRTS();
        rtsLine2 = serialPort2.GetRTS();
        
        ASSERT_FALSE(rtsLine1);
        ASSERT_FALSE(rtsLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetRTSGetCTS()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool ctsLine1 = false;
        bool ctsLine2 = false;

        serialPort1.SetRTS(true);
        serialPort2.SetRTS(true);

        ctsLine1 = serialPort1.GetCTS();
        ctsLine2 = serialPort2.GetCTS();

        ASSERT_TRUE(ctsLine1);
        ASSERT_TRUE(ctsLine2);
        
        serialPort1.SetRTS(false);
        serialPort2.SetRTS(false);

        ctsLine1 = serialPort1.GetCTS();
        ctsLine2 = serialPort2.GetCTS();

        ASSERT_FALSE(ctsLine1);
        ASSERT_FALSE(ctsLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetDTRGetDSR()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool dsrStatus1 = false;
        bool dsrStatus2 = false;

        serialPort1.SetDTR(true);
        serialPort2.SetDTR(true);

        dsrStatus1 = serialPort1.GetDSR();
        dsrStatus2 = serialPort1.GetDSR();

        ASSERT_TRUE(dsrStatus1);
        ASSERT_TRUE(dsrStatus2);

        serialPort1.SetDTR(false);
        serialPort2.SetDTR(false);

        dsrStatus1 = serialPort1.GetDSR();
        dsrStatus2 = serialPort1.GetDSR();

        ASSERT_FALSE(dsrStatus1);
        ASSERT_FALSE(dsrStatus2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortGetFileDescriptor()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialPort1.IsOpen());

        int fileDescriptor = serialPort1.GetFileDescriptor();
        ASSERT_GT(fileDescriptor, 0);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortReadCharBufferWriteCharBuffer()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        char writeByte1 = 'a';
        char writeByte2 = 'A';

        unsigned char readByte1  = 'b';
        unsigned char readByte2  = 'B';

        serialPort1.WriteByte(writeByte1);
        serialPort2.WriteByte(writeByte2);

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.Read(readByte2, 1, timeOutMilliseconds);
        serialPort2.Read(readByte1, 1, timeOutMilliseconds);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

        // try
        // {
        //     serialPort1.Read(readByte1, 1, 1);
        //     serialPort2.Read(readByte2, 1, 1);
        // }
        // catch(...)
        // {
        //     timeOutTestPass = true;
        // }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadDataBufferWriteDataBuffer()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        SerialPort::DataBuffer writeDataBuffer1;
        SerialPort::DataBuffer writeDataBuffer2;

        SerialPort::DataBuffer readDataBuffer1;
        SerialPort::DataBuffer readDataBuffer2;

        // Test using ASCII characters.
        for (size_t i = 48; i <= 122; i++)
        {
            writeDataBuffer1.push_back(i);
        }

        for (size_t i = 122; i >= 48; i--)
        {
            writeDataBuffer2.push_back(i);
        }

        serialPort1.Write(writeDataBuffer1);
        serialPort2.Write(writeDataBuffer2);

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());
        
        serialPort1.Read(readDataBuffer2, 75, timeOutMilliseconds);
        serialPort2.Read(readDataBuffer1, 75, timeOutMilliseconds);

        ASSERT_EQ(readDataBuffer1, writeDataBuffer1);
        ASSERT_EQ(readDataBuffer2, writeDataBuffer2);

        // try
        // {
        //     serialPort1.Read(readDataBuffer1, 1, 1);
        //     serialPort2.Read(readDataBuffer2, 1, 1);
        // }
        // catch(ReadTimeout)
        // {
        //     timeOutTestPass = true;
        // }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadStringWriteString()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        serialPort1.Write(writeString1);
        serialPort2.Write(writeString2);

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.Read(readString2, writeString2.size(), timeOutMilliseconds);
        serialPort2.Read(readString1, writeString1.size(), timeOutMilliseconds);

        ASSERT_EQ(readString1, writeString1);
        ASSERT_EQ(readString2, writeString2);

        // try
        // {
        //     serialPort1.Read(readString1, writeString1.size(), 1);
        //     serialPort2.Read(readString2, writeString2.size(), 1);
        // }
        // catch(ReadTimeout)
        // {
        //     timeOutTestPass = true;
        // }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadByteWriteByte()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        char writeByte1 = 'a';
        char writeByte2 = 'A';

        unsigned char readByte1  = 'b';
        unsigned char readByte2  = 'B';

        serialPort1.WriteByte(writeByte1);
        serialPort2.WriteByte(writeByte2);

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.ReadByte(readByte2, timeOutMilliseconds);
        serialPort2.ReadByte(readByte1, timeOutMilliseconds);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

        // try
        // {
        //     serialPort1.ReadByte(readByte1, 1);
        //     serialPort2.ReadByte(readByte2, 1);
        // }
        // catch(ReadTimeout)
        // {
        //     timeOutTestPass = true;
        // }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadLineWriteString()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort2.Open(TEST_SERIAL_PORT_2);
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        serialPort1.Write(writeString1 + '\n');
        serialPort2.Write(writeString2 + '\n');

        tcdrain(serialPort1.GetFileDescriptor());
        tcdrain(serialPort2.GetFileDescriptor());

        serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
        serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);

        ASSERT_EQ(readString1, writeString1 + '\n');
        ASSERT_EQ(readString2, writeString2 + '\n');
       
        // try
        // {
        //     serialPort1.ReadLine(readString2, '\n', 1);
        //     serialPort2.ReadLine(readString1, '\n', 1);
        // }
        // catch(ReadTimeout)
        // {
        //     timeOutTestPass = true;
        // }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialStreamToSerialPortReadWrite()
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

        size_t threadLoopStartTimeMicroseconds = getTimeInMicroSeconds();
        size_t timeElapsedMicroSeconds = 0;

        while (timeElapsedMicroSeconds < threadTimeOutMicroseconds)
        {
            serialStream1 << writeString1 << std::endl;
            getline(serialStream1, readString2);
            ASSERT_EQ(readString2, writeString2);

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - threadLoopStartTimeMicroseconds;
        }

        serialStream1.Close();
        return;
    }

    void serialStream2ThreadLoop()
    {
        serialStream2.Open(TEST_SERIAL_PORT_2);
        serialStream2.SetBaudRate(BaudRate::BAUD_115200);

        size_t threadLoopStartTimeMicroseconds = getTimeInMicroSeconds();
        size_t timeElapsedMicroSeconds = 0;

        while (timeElapsedMicroSeconds < threadTimeOutMicroseconds)
        {
            serialStream2 << writeString2 << std::endl;

            getline(serialStream2, readString1);

            ASSERT_EQ(readString1, writeString1);

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - threadLoopStartTimeMicroseconds;
        }

        serialStream2.Close();
        return;
    }

    void serialPort1ThreadLoop()
    {
        serialPort1.Open(TEST_SERIAL_PORT_1);
        serialPort1.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        size_t threadLoopStartTimeMicroseconds = getTimeInMicroSeconds();
        size_t timeElapsedMicroSeconds = 0;

        while (timeElapsedMicroSeconds < threadTimeOutMicroseconds)
        {
            serialPort1.Write(writeString1 + '\n');
            tcdrain(serialPort1.GetFileDescriptor());

            serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
            ASSERT_EQ(readString2, writeString2 + '\n');

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - threadLoopStartTimeMicroseconds;
        }

        serialPort1.Close();
        return;
    }

    void serialPort2ThreadLoop()
    {
        serialPort2.Open(TEST_SERIAL_PORT_2);
        serialPort2.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        size_t threadLoopStartTimeMicroseconds = getTimeInMicroSeconds();
        size_t timeElapsedMicroSeconds = 0;

        while (timeElapsedMicroSeconds < threadTimeOutMicroseconds)
        {
            serialPort2.Write(writeString2 + '\n');

            tcdrain(serialPort2.GetFileDescriptor());

            serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);

            ASSERT_EQ(readString1, writeString1 + '\n');

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - threadLoopStartTimeMicroseconds;
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
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushInputBuffer)
{
    SCOPED_TRACE("Serial Port FlushInputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushInputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Port FlushOutputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushOutputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushIOBuffers)
{
    SCOPED_TRACE("Serial Port FlushIOBuffers() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushIOBuffers();
    }
}

TEST_F(LibSerialTest, testSerialStreamIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}
TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadWriteByte)
{
    SCOPED_TRACE("Serial Stream Read() and WriteByte() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamReadWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetLineWriteString)
{
    SCOPED_TRACE("Serial Stream GetLine() and Write(string) Test");
        
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetLineWriteString();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetWriteByte)
{
    SCOPED_TRACE("Serial Stream Get() and WriteByte() Test");
        
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetWriteByte();
    }
}


// //------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushInputBuffer)
{
    SCOPED_TRACE("Serial Port FlushInputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushInputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Port FlushOutputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushOutputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushIOBuffers)
{
    SCOPED_TRACE("Serial Port FlushIOBuffers() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushIOBuffers();
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDTR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetRTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortSetRTSGetCTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetCTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetRTSGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortSetDTRGetDSR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDSR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetDTRGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortGetFileDescriptor)
{
    SCOPED_TRACE("Serial Port GetFileDescriptor() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortGetFileDescriptor();
    }
}

TEST_F(LibSerialTest, testSerialPortReadCharBufferWriteCharBuffer)
{
    SCOPED_TRACE("Serial Port Read(unsigned char) and Write(unsigned char) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadCharBufferWriteCharBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortReadDataBufferWriteDataBuffer)
{
    SCOPED_TRACE("Serial Port Read(DataBuffer) and Write(DataBuffer) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadDataBufferWriteDataBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortReadStringWriteString)
{
    SCOPED_TRACE("Serial Port Read(string) and Write(string) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadStringWriteString();
    }
}

TEST_F(LibSerialTest, testSerialPortReadByteWriteByte)
{
    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadByteWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialPortReadLineWriteString)
{
    SCOPED_TRACE("Serial Port ReadLine() and Write(string) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadLineWriteString();
    }
}


//----------------- Serial Stream to Serial Port Unit Test ------------------//

TEST_F(LibSerialTest, testSerialStreamToSerialPortReadWrite)
{
    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamToSerialPortReadWrite();
    }
}


//----------------------- Multiple Thread Unit Tests ------------------------//

TEST_F(LibSerialTest, testMultiThreadSerialStreamReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Stream Communication.");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testMultiThreadSerialStreamReadWrite();
    }
}

TEST_F(LibSerialTest, testMultiThreadSerialPortReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Port Communication.");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testMultiThreadSerialPortReadWrite();
    }
}
