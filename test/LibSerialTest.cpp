/**
 * @file LibSerialTest.cpp
 * @copyright LibSerial
 */

#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <unistd.h>


#include <gtest/gtest.h>
#include <SerialPort.h>
#include <SerialStream.h>

// Default Serial Port and Baud Rate.
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

using namespace LibSerial;

class LibSerialTest
    : public ::testing::Test
{
public:
    LibSerialTest() : serialPort1(TEST_SERIAL_PORT_1), serialPort2(TEST_SERIAL_PORT_2) {}

protected:
    SerialPort serialPort1;
    SerialPort serialPort2;

    /** @brief The pthread mutex lock to lock data and avoid race conditions. */
    pthread_mutex_t serialPort1CommunicationThreadMutex;
    pthread_mutex_t serialPort2CommunicationThreadMutex;
    pthread_mutex_t serialStream1CommunicationThreadMutex;
    pthread_mutex_t serialStream2CommunicationThreadMutex;

    /** @brief The pthread ID for the communicationsThread. */
    pthread_t serialPort1CommunicationThreadID;
    pthread_t serialPort2CommunicationThreadID;
    pthread_t serialStream1CommunicationThreadID;
    pthread_t serialStream2CommunicationThreadID;

    /** @brief A flag to indicate if the thread is currently running. */
    bool serialPort1ThreadRunning;
    bool serialPort2ThreadRunning;
    bool serialStream1ThreadRunning;
    bool serialStream2ThreadRunning;

    virtual void SetUp()
    {
        serialPort1ThreadRunning = false;
        serialPort2ThreadRunning = false;

        writeString1 = "Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)";
        writeString2 = "The universally interesting man is universally interested. - William Dean Howells";

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


    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamReadWrite()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream1.SetBaudRate(baudRates[16]);
        serialStream2.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate1 = serialStream1.GetBaudRate();
        BaudRate baudRate2 = serialStream2.GetBaudRate();
        
        ASSERT_EQ(baudRate1, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

        serialStream1 << writeString1 << std::endl;
        getline(serialStream2, readString1);
        ASSERT_EQ(readString1, writeString1);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());

        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        serialStream2 << writeString2 << std::endl;
        getline(serialStream1, readString2);
        ASSERT_EQ(readString2, writeString2);

        writeByte = 'a';
        
        serialStream1.write(&writeByte, 1);
        serialStream1 << writeByte << std::endl;

        serialStream2.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream2.write(&writeByte, 1);
        serialStream2 << writeByte << std::endl;
        
        serialStream1.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());

        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        writeByte = 'A';
        
        serialStream1.write(&writeByte, 1);
        serialStream1 << writeByte << std::endl;
        
        serialStream2.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        serialStream2.write(&writeByte, 1);
        serialStream2 << writeByte << std::endl;
        
        serialStream1.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
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

        // @TODO - Why don't the smaller Character Size values work?
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


    //----------------------- Serial Port Unit Tests ------------------------//

    void testSerialPortOpenClose()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortReadWrite(const int timeOutMilliseconds)
    {
        serialPort1.Open();
        serialPort2.Open();
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.SetBaudRate(baudRates[16]);
        serialPort2.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate1 = serialPort1.GetBaudRate();
        BaudRate baudRate2 = serialPort2.GetBaudRate();
        
        ASSERT_EQ(baudRate1, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

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
    
        unsigned char writeByte;
        unsigned char readByte;
        
        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
        
        serialPort1.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

        serialPort2.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);

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

    void testSerialPortIsDataAvailableTest()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        unsigned char writeByte;
        unsigned char readByte;

        writeByte = 'A';

        serialPort1.WriteByte(writeByte);
        tcdrain(serialPort1.GetFileDescriptor());
        
        usleep(2000);

        ASSERT_TRUE(serialPort2.IsDataAvailable());

        serialPort2.ReadByte(readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
                
        usleep(2000);

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
        serialPort1.Open();
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
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        // @TODO - Why don't the smaller CharSize values work?
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
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        // @TODO - FLOW_CONTROL_SOFT flow control is not valid.
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
        serialPort1.Open();
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
        StopBits numberOfStopBits;

        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetNumberOfStopBits(stopBits[0]);
        numberOfStopBits = serialPort1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[0]);

        serialPort1.SetNumberOfStopBits(stopBits[1]);
        numberOfStopBits = serialPort1.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[1]);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetDTR()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetDtr(true);
        bool dtrLine = serialPort1.GetDtr();
        ASSERT_TRUE(dtrLine);

        serialPort1.SetDtr(false);
        dtrLine = serialPort1.GetDtr();
        ASSERT_FALSE(dtrLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetRts(true);
        bool rtsLine = serialPort1.GetRts();
        ASSERT_TRUE(rtsLine);

        serialPort1.SetRts(false);
        rtsLine = serialPort1.GetRts();
        ASSERT_FALSE(rtsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetCTS()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        bool ctsLine = serialPort1.GetCts();
        ASSERT_FALSE(ctsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetDSR()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        bool dsrLine = serialPort1.GetDsr();
        ASSERT_FALSE(dsrLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetVMin()
    {
        serialPort1.Open();
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
        serialPort1.Open();
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

    void testSerialStreamToSerialPortReadWrite(const int timeOutMilliseconds)
    {
        serialPort1.Open();
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

    void testMultiThreadedSerialStreamReadWrite()
    {
        engageSerialStreamCommunicationThreads();
        
        while(serialStream1ThreadRunning && serialStream2ThreadRunning)
        {
            usleep(1000);
        }
    }

    void testMultiThreadedSerialPortReadWrite()
    {
        engageSerialPortCommunicationThreads();
        
        while(serialPort1ThreadRunning && serialPort2ThreadRunning)
        {
            usleep(1000);
        }
    }

    long unsigned int getTimeInMicroSeconds()
    {
        std::chrono::high_resolution_clock::duration timeNow = 
            std::chrono::high_resolution_clock::now().time_since_epoch();

        return std::chrono::duration_cast<std::chrono::microseconds>(timeNow).count();
    }

    void serialStream1CommunicationThreadLoop()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream1.SetBaudRate(BaudRate::BAUD_115200);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeRemainingMicroSeconds = 0;

            while (timeElapsedMicroSeconds < 250000000)
            {
                loopStartTimeMicroseconds = getTimeInMicroSeconds();

                serialStream1 << writeString1 << std::endl;
                getline(serialStream1, readString2);
                ASSERT_EQ(readString2, writeString2);

                timeRemainingMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;

                usleep(timeRemainingMicroSeconds);

                timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
            }

        serialStream1.Close();
        serialStream1ThreadRunning = false;
    }

    void serialStream2CommunicationThreadLoop()
    {
        serialStream2.Open(TEST_SERIAL_PORT_1);
        serialStream2.SetBaudRate(BaudRate::BAUD_115200);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeRemainingMicroSeconds = 0;

            while (timeElapsedMicroSeconds < 250000000)
            {
                loopStartTimeMicroseconds = getTimeInMicroSeconds();

                serialStream2 << writeString2 << std::endl;
                getline(serialStream2, readString1);
                ASSERT_EQ(readString1, writeString1);

                timeRemainingMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;

                usleep(timeRemainingMicroSeconds);

                timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
            }

        serialStream2.Close();
        serialStream2ThreadRunning = false;
    }

    void serialPort1CommunicationThreadLoop()
    {
        serialPort1.Open();
        serialPort1.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeRemainingMicroSeconds = 0;
        long unsigned int timeOutMilliseconds = 25;

            while (timeElapsedMicroSeconds < 250000000)
            {
                loopStartTimeMicroseconds = getTimeInMicroSeconds();

                serialPort1.Write(writeString1 + '\n');
                tcdrain(serialPort1.GetFileDescriptor());

                serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
                ASSERT_EQ(readString2, writeString2 + '\n');

                timeRemainingMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;

                usleep(timeRemainingMicroSeconds);

                timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
            }

        serialPort1.Close();
        serialPort1ThreadRunning = false;
    }

    void serialPort2CommunicationThreadLoop()
    {
        serialPort2.Open();
        serialPort2.SetBaudRate(BaudRate::BAUD_115200);
        tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

        long unsigned int loopStartTimeMicroseconds = 0;
        long unsigned int timeElapsedMicroSeconds = 0;
        long unsigned int timeRemainingMicroSeconds = 0;
        long unsigned int timeOutMilliseconds = 250;

        while (timeElapsedMicroSeconds < 250000000)
        {
            loopStartTimeMicroseconds = getTimeInMicroSeconds();

            serialPort2.Write(writeString2 + '\n');
            tcdrain(serialPort2.GetFileDescriptor());

            serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);
            ASSERT_EQ(readString1, writeString1 + '\n');

            timeRemainingMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;

            usleep(timeRemainingMicroSeconds);

            timeElapsedMicroSeconds = getTimeInMicroSeconds() - loopStartTimeMicroseconds;
        }

        serialPort2.Close();
        serialPort2ThreadRunning = false;
    }

    void engageSerialPortCommunicationThreads()
    {
        pthread_mutex_init(&serialPort1CommunicationThreadMutex, NULL);
        pthread_create(&serialPort1CommunicationThreadID, NULL, &startSerialPort1CommunicationThread, this);

        pthread_mutex_init(&serialPort2CommunicationThreadMutex, NULL);
        pthread_create(&serialPort2CommunicationThreadID, NULL, &startSerialPort2CommunicationThread, this);

        serialPort1ThreadRunning = true;
        serialPort2ThreadRunning = true;
        return;
    }

    void engageSerialStreamCommunicationThreads()
    {
        pthread_mutex_init(&serialStream1CommunicationThreadMutex, NULL);
        pthread_create(&serialStream1CommunicationThreadID, NULL, &startSerialStream1CommunicationThread, this);

        pthread_mutex_init(&serialStream2CommunicationThreadMutex, NULL);
        pthread_create(&serialStream2CommunicationThreadID, NULL, &startSerialStream2CommunicationThread, this);

        serialStream1ThreadRunning = true;
        serialStream2ThreadRunning = true;
        return;
    }

    static void* startSerialPort1CommunicationThread(void *args)
    {
        LibSerialTest* libSerialTest  = (LibSerialTest*) args;
        libSerialTest->serialPort1CommunicationThreadLoop();
        return NULL;
    }

    static void* startSerialPort2CommunicationThread(void *args)
    {
        LibSerialTest* libSerialTest  = (LibSerialTest*) args;
        libSerialTest->serialPort2CommunicationThreadLoop();
        return NULL;
    }

    static void* startSerialStream1CommunicationThread(void *args)
    {
        LibSerialTest* libSerialTest  = (LibSerialTest*) args;
        libSerialTest->serialStream1CommunicationThreadLoop();
        return NULL;
    }

    static void* startSerialStream2CommunicationThread(void *args)
    {
        LibSerialTest* libSerialTest  = (LibSerialTest*) args;
        libSerialTest->serialStream2CommunicationThreadLoop();
        return NULL;
    }


    BaudRate        baudRates[25];
    CharacterSize   characterSizes[4];
    FlowControl     flowControlTypes[3];
    Parity          parityTypes[3];
    StopBits        stopBits[2];

    SerialStream serialStream1;
    SerialStream serialStream2;

    std::string readString1;
    std::string writeString1;

    std::string readString2;
    std::string writeString2;

    char writeByte;
    char readByte;
};



//------------------------ Serial Stream Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open and Close Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadWrite)
{
    SCOPED_TRACE("Serial Stream Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamReadWrite();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream Set and Get Baud Rate Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream Set and Get Character Size Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream Set and Get Parity Type Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream Set and Get Flow Control Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream Set and Get Stop Bits Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Port Set and Get vMin Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port Set and Get vTime Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVTime();
    }
}


//------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open and Close Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortReadWrite)
{
    int timeOutMilliseconds = 30;

    SCOPED_TRACE("Serial Port Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadWrite(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port Is Data Available Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port Set and Get Baud Rate Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port Set and Get Character Size Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port Set and Get Parity Type Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port Set and Get Flow Control Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port Set and Get Stop Bits Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetStopBits();
    }
}
TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port Set and Get DTR Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port Set and Get RTS Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetCTS)
{
    SCOPED_TRACE("Serial Port Get CTS Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetDSR)
{
    SCOPED_TRACE("Serial Port Get DSR Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port Set and Get vMin Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port Set and Get vTime Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVTime();
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

TEST_F(LibSerialTest, testMultiThreadedSerialPortReadWrite)
{
    SCOPED_TRACE("Test Multi-Threaded Serial Port Communication.");
    testMultiThreadedSerialPortReadWrite();
}

TEST_F(LibSerialTest, testMultiThreadedSerialStreamReadWrite)
{
    SCOPED_TRACE("Test Multi-Threaded Serial Stream Communication.");
    testMultiThreadedSerialStreamReadWrite();
}
