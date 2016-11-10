/******************************************************************************
 *   @file SerialPort.cpp                                                     *
 *   Copyright (C) 2004 by Manish Pagey                                       *
 *   crayzeewulf@users.sourceforge.net                                        *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU General Public License as published by     *
 *   the Free Software Foundation; either version 2 of the License, or        *
 *   (at your option) any later version.                                      *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU General Public License for more details.                             *
 *                                                                            *
 *   You should have received a copy of the GNU General Public License        *
 *   along with this program; if not, write to the                            *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#include "SerialPort.h"
#include "PosixSignalDispatcher.h"
#include "PosixSignalHandler.h"

#include <cstring>
#include <fcntl.h>
#include <queue>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>


namespace
{
    /**
     * Return the difference between the two specified timeval values.
     * This method subtracts secondOperand from firstOperand and returns
     * the result as a timeval. The time represented by firstOperand must
     * be later than the time represented by secondOperand. Otherwise,
     * the result of this operator may be undefined.
     */
    const timeval
    operator-(const timeval& firstOperand,
              const timeval& secondOperand);
}

namespace LibSerial 
{
    class SerialPort::Implementation : public PosixSignalHandler
    {
    public:
        /**
         * @brief Default Constructor.
         * @param serialPortName The serial port name to be opened.
         */
        Implementation(const std::string& serialPortName);

        /**
         * @brief Destructor.
         */
        ~Implementation();

        /**
         * @brief Opens the serial port.
         */
        void Open();

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen();

        /**
         * @brief Sets the baud rate for the serial port to the specified value
         * @param baudRate The baud rate to be set for the serial port.
         */
        void SetBaudRate(const BaudRate& baudRate);

        /**
         * @brief Gets the current baud rate for the serial port.
         * @return Returns the baud rate.
         */
        BaudRate GetBaudRate();

        /**
         * @brief Sets the character size for the serial port.
         * @param characterSize The character size to be set.
         */
        void SetCharacterSize(const CharacterSize& characterSize);

        /**
         * @brief Gets the character size being used for serial communication.
         * @return Returns the current character size. 
         */
        CharacterSize GetCharacterSize();

        /**
         * @brief Sets flow control for the serial port.
         * @param flowControlType The flow control type to be set.
         */
        void SetFlowControl(const FlowControl& flowControl);

        /**
         * @brief Get the current flow control setting.
         * @return Returns the flow control type of the serial port.
         */
        FlowControl GetFlowControl();

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity& parityType);

        /**
         * @brief Gets the parity type for the serial port.
         * @return Returns the parity type.
         */
        Parity GetParity();

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param numberOfStopBits The number of stop bits to set.
         */
        void SetNumberOfStopBits(const StopBits& numberOfStopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetNumberOfStopBits();

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @param vmin the number of minimum characters to be set.
         */
        void SetVMin(const short& vmin);

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for non-canonical reads.
         */
        short GetVMin();

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         */
        void SetVTime(const short& vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

        /**
         * @brief Determines if data is available at the serial port.
         */
        bool IsDataAvailable();

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataBuffer on
         *        return from this method.
         * @param dataBuffer The data buffer to place serial data into.
         * @param numOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         * @return Returns the number of bytes read.
         */
        int Read(SerialPort::DataBuffer& dataBuffer,
                 const unsigned int&     numOfBytes,
                 const unsigned int&     msTimeout);

        /**
         * @brief Reads a single byte from the serial port.
         *        If no data is available within the specified number
         *        of milliseconds (msTimeout), then this method will
         *        throw a ReadTimeout exception. If msTimeout is 0,
         *        then this method will block until data is available.
         * @param msTimeout The timeout period in milliseconds.
         * @return Returns the number of bytes read.
         */
        int ReadByte(unsigned char&      charBuffer, 
                     const unsigned int& msTimeout = 0);

        /**
         * @brief Reads a line of characters from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until a line terminator is received.
         *        If a line terminator is read, a string will be returned,
         *        however, if the timeout is reached, an exception will be thrown
         *        and all previously read data will be lost.
         * @param dataString The data string read from the serial port.
         * @param lineTerminator The line termination character to specify the
         *        end of a line.
         * @param msTimeout The timeout value to return if a line termination
         *        character is not read.
         * @return Returns the number of bytes read.
         */
        int ReadLine(std::string&        dataString,
                     const char&         lineTerminator = '\n',
                     const unsigned int& msTimeout = 0);

        /**
         * @brief Writes a single byte to the serial port.
         * @param dataByte The byte to be written to the serial port.
         */
        void WriteByte(const unsigned char& dataByte);

        /**
         * @brief Writes a DataBuffer vector to the serial port.
         * @param dataBuffer The DataBuffer vector to be written to the serial port.
         */
        void Write(const SerialPort::DataBuffer& dataBuffer);

        /**
         * @brief Writes a DataBuffer vector to the serial port.
         * @param dataBuffer The DataBuffer vector to be written to the serial port.
         * @param bufferSize The number of bytes to be written to the serial port.
         */
        void Write(const unsigned char* dataBuffer,
                   const unsigned int&   bufferSize);

        /**
         * @brief Sets the serial port DTR line status.
         * @param dtrState The state to set the DTR line
         */
        void SetDtr(const bool& dtrState);

        /**
         * @brief Gets the serial port DTR line status.
         */
        bool GetDtr();

        /**
         * @brief Sets the serial port RTS line status.
         * @param dtrState The state to set the RTS line
         */
        void SetRts(const bool& rtsState);

        /**
         * @brief Gets the serial port RTS line status.
         */
        bool GetRts();

        /**
         * @brief Gets the serial port CTS line status.
         */
        bool GetCts();

        /**
         * @brief Gets the serial port DSR line status.
         */
        bool GetDsr();

        /**
         * @brief Gets the serial port file descriptor.
         */
        int GetFileDescriptor();

        /*
         * @brief This method must be defined by all subclasses of
         *        PosixSignalHandler.
         */
        void HandlePosixSignal(const int& signalNumber) override;
    private:
        /**
         * @brief Name of the serial port. On POSIX systems this is the name of
         *        the device file.
         */
        std::string mSerialPortName {};

        /**
         * @brief Flag that indicates whether the serial port is currently open.
         */
        bool mIsOpen {false};

        /**
         * @brief The file descriptor corresponding to the serial port.
         */
        int mFileDescriptor {-1};

        /**
         * @brief Serial port settings are saved into this variable immediately
         *        after the port is opened. These settings are restored when the
         *        serial port is closed.
         */
        termios mOldPortSettings {};

        /**
         * @brief Circular buffer used to store the received data. This is done
         *        asynchronously and helps prevent overflow of the corresponding 
         *        tty's input buffer.
         * 
         * @TODO: The size of this buffer is allowed to increase indefinitely. If 
         *        data keeps arriving at the serial port and is never read then this 
         *        buffer will continue occupying more and more memory. We need to put a 
         *        cap on the size of this buffer. It might even be worth providing a 
         *        method to set the size of this buffer.  
         */
        std::queue<unsigned char> mInputBuffer {};

        /**
         * @brief In order to be thread safe, a second queue is implemented to store
         *        bytes which are received while the queue is locked by the
         *        ReadByte method. The content must be transfered to mInputBuffer
         *        before further bytes are stored into it.
         */
        std::queue<unsigned char> mShadowInputBuffer {};

        /**
         * @brief Mutex to control threaded access to mInputBuffer
         */
        pthread_mutex_t mQueueMutex {};

        /**
         * @brief Set the specified modem control line to the specified value. 
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @param lineState State of the modem line after successful
         *        call to this method.
         */
        void
        SetModemControlLine(const int& modemLine,
                            const bool& lineState);

        /**
         * @brief Get the current state of the specified modem control line.
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @return True if the specified line is currently set and false
         *         otherwise.
         */
        bool
        GetModemControlLine(const int& modemLine);
    };

    SerialPort::SerialPort(const std::string& serialPortName)
        : mImpl(new Implementation(serialPortName))
    {
        // Empty
    }

    SerialPort::~SerialPort()
    throw()
    {
        // Close the serial port if it is open.
        if (this->IsOpen())
        {
            this->Close();
        }

        return;
    }

    void
    SerialPort::Open(const BaudRate&      baudRate,
                     const CharacterSize& characterSize,
                     const FlowControl&   flowControlType,
                     const Parity&        parityType,
                     const StopBits&      stopBits)
    {
        // Open the serial port.
        mImpl->Open();

        // Set the various parameters of the serial port if it is open.
        this->SetBaudRate(baudRate);
        this->SetCharacterSize(characterSize);
        this->SetFlowControl(flowControlType);
        this->SetParity(parityType);
        this->SetNumberOfStopBits(stopBits);
        return;
    }

    void
    SerialPort::Close()
    {
        mImpl->Close();
        return;
    }

    bool
    SerialPort::IsOpen()
    {
        return mImpl->IsOpen();
    }

    void
    SerialPort::SetBaudRate(const BaudRate& baudRate)
    {
        mImpl->SetBaudRate(baudRate);
        return;
    }

    BaudRate
    SerialPort::GetBaudRate()
    {
        return mImpl->GetBaudRate();
    }


    void
    SerialPort::SetCharacterSize(const CharacterSize& characterSize)
    {
        mImpl->SetCharacterSize(characterSize);
        return;
    }

    CharacterSize
    SerialPort::GetCharacterSize()
    {
        return mImpl->GetCharacterSize();
    }

    void
    SerialPort::SetFlowControl(const FlowControl& flowControl)
    {
        mImpl->SetFlowControl(flowControl);
        return;
    }

    FlowControl
    SerialPort::GetFlowControl()
    {
        return mImpl->GetFlowControl();
    }

    void
    SerialPort::SetParity(const Parity& parityType)
    {
        mImpl->SetParity( parityType );
        return;
    }

    Parity
    SerialPort::GetParity()
    {
        return mImpl->GetParity();
    }

    void
    SerialPort::SetNumberOfStopBits(const StopBits& numberOfStopBits)
    {
        mImpl->SetNumberOfStopBits(numberOfStopBits);
        return;
    }

    StopBits
    SerialPort::GetNumberOfStopBits()
    {
        return mImpl->GetNumberOfStopBits();
    }

    void
    SerialPort::SetVMin(const short& vmin)
    {
        mImpl->SetVMin(vmin);
        return;
    }

    short
    SerialPort::GetVMin()
    {
        return mImpl->GetVMin();
    }

    void
    SerialPort::SetVTime(const short& vtime)
    {
        mImpl->SetVTime(vtime);
        return;
    }

    short
    SerialPort::GetVTime()
    {
        return mImpl->GetVTime();
    }

    bool
    SerialPort::IsDataAvailable()
    {
        return mImpl->IsDataAvailable();
    }

    int
    SerialPort::Read(SerialPort::DataBuffer&  dataBuffer,
                     const unsigned int&      numOfBytes,
                     const unsigned int&      msTimeout)
    {
        return mImpl->Read(dataBuffer,
                           numOfBytes,
                           msTimeout);
    }

    int
    SerialPort::ReadByte(unsigned char&      charBuffer,
                         const unsigned int& msTimeout)
    {
        return mImpl->ReadByte(charBuffer,
                               msTimeout);
    }

    int
    SerialPort::ReadLine(std::string&        dataString,
                         const char&         lineTerminator,
                         const unsigned int& msTimeout)
    {
        return mImpl->ReadLine(dataString,
                               lineTerminator,
                               msTimeout);
    }

    void
    SerialPort::WriteByte(const unsigned char& dataByte)
    {
        mImpl->WriteByte(dataByte);
        return;
    }

    void
    SerialPort::Write(const DataBuffer& dataBuffer)
    {
        mImpl->Write(dataBuffer);
        return;
    }

    void
    SerialPort::Write(const std::string& dataString)
    {
        mImpl->Write(reinterpret_cast<const unsigned char*>(dataString.c_str()),
                     dataString.length());
        return;
    }

    void
    SerialPort::SetDtr(const bool& dtrState)
    {
        mImpl->SetDtr(dtrState);
        return;
    }

    bool
    SerialPort::GetDtr() 
    {
        return mImpl->GetDtr();
    }

    void
    SerialPort::SetRts(const bool& rtsState)
    {
        mImpl->SetRts(rtsState);
        return;
    }

    bool
    SerialPort::GetRts() 
    {
        return mImpl->GetRts();
    }


    bool
    SerialPort::GetCts() 
    {
        return mImpl->GetCts();
    }

    bool
    SerialPort::GetDsr() 
    {
        return mImpl->GetDsr();
    }

    int
    SerialPort::GetFileDescriptor()
    {
        return mImpl->GetFileDescriptor();
    }

    /** ------------------------------------------------------------ */
    inline
    SerialPort::Implementation::Implementation(const std::string& serialPortName)
        : mSerialPortName(serialPortName)
    {
        //Initializing the mutex
        if (pthread_mutex_init(&mQueueMutex, NULL) != 0)
        {
            throw std::runtime_error(ERR_MSG_PTHREAD_MUTEX_ERROR);
        }
    }

    inline
    SerialPort::Implementation::~Implementation()
    {
        // Close the serial port if it is open.
        if (this->IsOpen())
        {
            this->Close();
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::Open()
    {
        // Throw an exception if the port is already open.
        if (this->IsOpen())
        {
            throw AlreadyOpen(ERR_MSG_PORT_ALREADY_OPEN);
        }

        // Try to open the serial port and throw an exception if we are
        // not able to open it.

        // :FIXME: Exception thrown by this method after opening the
        // serial port might leave the port open even though mIsOpen
        // is false. We need to close the port before throwing an
        // exception or close it next time this method is called before
        // calling open() again.
        mFileDescriptor = open(mSerialPortName.c_str(),
                               O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (mFileDescriptor < 0)
        {
            close(mFileDescriptor);
            throw OpenFailed(strerror(errno));
        }

        PosixSignalDispatcher& signal_dispatcher = PosixSignalDispatcher::Instance();
        signal_dispatcher.AttachHandler(SIGIO,
                                        *this);

        // Direct all SIGIO and SIGURG signals for the port to the current process.
        if (fcntl(mFileDescriptor,
                  F_SETOWN,
                  getpid()) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Enable asynchronous I/O with the serial port.
        if (fcntl(mFileDescriptor,
                  F_SETFL,
                  FASYNC) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Save the current settings of the serial port so they can be
        // restored when the serial port is closed.
        if (tcgetattr(mFileDescriptor,
                      &mOldPortSettings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Start assembling the new port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));

        // Enable the receiver (CREAD) and ignore modem control lines (CLOCAL).
        port_settings.c_cflag |= CREAD | CLOCAL;

        // Set the VMIN and VTIME parameters to zero by default. VMIN is
        // the minimum number of characters for non-canonical read and
        // VTIME is the timeout in deciseconds for non-canonical
        // read. Setting both of these parameters to zero implies that a
        // read will return immediately only giving the currently
        // available characters.
        port_settings.c_cc[VMIN] = 0;
        port_settings.c_cc[VTIME] = 0;

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Flush the input and output buffers associated with the port.
        if (tcflush(mFileDescriptor,
                    TCIOFLUSH) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Set the serial port open flag.
        mIsOpen = true;

        return;
    }

    inline
    void
    SerialPort::Implementation::Close()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        PosixSignalDispatcher& signal_dispatcher = PosixSignalDispatcher::Instance();
        signal_dispatcher.DetachHandler(SIGIO,
                                        *this);

        // Restore the old settings of the port.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &mOldPortSettings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        // Close the serial port file descriptor.
        close(mFileDescriptor);

        // Set the serial port open flag.
        mIsOpen = false;

        return;
    }

    inline
    bool
    SerialPort::Implementation::IsOpen()
    {
        return mIsOpen;
    }

    inline
    void
    SerialPort::Implementation::SetBaudRate(const BaudRate& baudRate)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }
        
        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the baud rate for both input and output.
        if (cfsetispeed(&port_settings, (speed_t)baudRate) < 0 ||
            cfsetospeed(&port_settings, (speed_t)baudRate) < 0 )
        {
            // If any of the settings fail, we abandon this method.
            throw UnsupportedBaudRate(ERR_MSG_UNSUPPORTED_BAUD);
        }

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw UnsupportedBaudRate(strerror(errno));
        }

        return;
    }

    inline
    BaudRate
    SerialPort::Implementation::GetBaudRate()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Obtain the input baud rate from the current settings.
        return BaudRate(cfgetispeed(&port_settings));
    }

    inline
    void
    SerialPort::Implementation::SetCharacterSize(const CharacterSize& characterSize)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the character size.
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= (tcflag_t)characterSize;

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    CharacterSize
    SerialPort::Implementation::GetCharacterSize()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Read the character size from the setttings.
        return CharacterSize(port_settings.c_cflag & CSIZE);
    }

    inline
    void
    SerialPort::Implementation::SetFlowControl(const FlowControl& flowControl)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the flow control.
        switch(flowControl)
        {
        case FlowControl::FLOW_CONTROL_HARD:
            port_settings.c_cflag |= CRTSCTS;
            break;
        case FlowControl::FLOW_CONTROL_NONE:
            port_settings.c_cflag &= ~(CRTSCTS);
            break;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_FLOW_CONTROL);
            break;
        }

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    FlowControl
    SerialPort::Implementation::GetFlowControl()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // If CRTSCTS is set then we are using hardware flow
        // control. Otherwise, we are not using any flow control.
        if (port_settings.c_cflag & CRTSCTS)
        {
            return FlowControl::FLOW_CONTROL_HARD;
        }

        return FlowControl::FLOW_CONTROL_NONE;
    }

    inline
    void
    SerialPort::Implementation::SetParity(const Parity& parityType)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the parity type depending on the specified parameter.
        switch(parityType)
        {
        case Parity::PARITY_EVEN:
            port_settings.c_cflag |= PARENB;
            port_settings.c_cflag &= ~PARODD;
            port_settings.c_iflag |= INPCK;
            break;
        case Parity::PARITY_ODD:
            port_settings.c_cflag |= (PARENB | PARODD);
            port_settings.c_iflag |= INPCK;
            break;
        case Parity::PARITY_NONE:
            port_settings.c_cflag &= ~(PARENB);
            port_settings.c_iflag |= IGNPAR;
            break;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_PARITY);
            break;
        }

        // Apply the modified port settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    Parity
    SerialPort::Implementation::GetParity()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Get the parity type from the current settings.
        if (port_settings.c_cflag & PARENB)
        {
            // Parity is enabled. Lets check if it is odd or even.
            if (port_settings.c_cflag & PARODD)
            {
                return Parity::PARITY_ODD;
            }
            else
            {
                return Parity::PARITY_EVEN;
            }
        }

        // Parity is disabled.
        return Parity::PARITY_NONE;
    }

    inline
    void
    SerialPort::Implementation::SetNumberOfStopBits(const StopBits& numberOfStopBits)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the number of stop bits.
        switch(numberOfStopBits)
        {
        case StopBits::STOP_BITS_1:
            port_settings.c_cflag &= ~(CSTOPB);
            break;
        case StopBits::STOP_BITS_2:
            port_settings.c_cflag |= CSTOPB;
            break;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_STOP_BITS);
            break;
        }

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    StopBits
    SerialPort::Implementation::GetNumberOfStopBits()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // If CSTOPB is set then we are using two stop bits, otherwise we
        // are using 1 stop bit.
        if (port_settings.c_cflag & CSTOPB)
        {
            return StopBits::STOP_BITS_2;
        }

        return StopBits::STOP_BITS_1;
    }

    inline
    void 
    SerialPort::Implementation::SetVMin(const short& vmin)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (vmin < 0 || vmin > 255)
        {
            throw std::invalid_argument(strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_cc[VMIN] = (cc_t)vmin;

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    short 
    SerialPort::Implementation::GetVMin()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return port_settings.c_cc[VMIN];
    }

    inline
    void 
    SerialPort::Implementation::SetVTime(const short& vtime)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (vtime < 0 || vtime > 255)
        {
            throw std::invalid_argument(strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_cc[VTIME] = (cc_t)vtime;

        // Apply the modified settings.
        if (tcsetattr(mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::invalid_argument(strerror(errno));
        }

        return;
    }

    inline
    short 
    SerialPort::Implementation::GetVTime() 
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return port_settings.c_cc[VTIME];
    }

    inline
    bool
    SerialPort::Implementation::IsDataAvailable()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        pthread_mutex_lock(&mQueueMutex);
        bool dataAvailableStatus = !mInputBuffer.empty();
        pthread_mutex_unlock(&mQueueMutex);

        return dataAvailableStatus;
    }

    inline
    int
    SerialPort::Implementation::Read(SerialPort::DataBuffer& dataBuffer,
                                     const unsigned int&     numOfBytes,
                                     const unsigned int&     msTimeout)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        unsigned int elapsed_ms;
        unsigned int remaining_ms;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;

        // Throw an exception if we are unable to read the current time.  
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }
        
        // Empty the data buffer.
        dataBuffer.resize(0);
        unsigned char nextChar = 0;
        size_t bytesRead = 0;

        if (0 == numOfBytes)
        {
            // Read all available data if numOfBytes is zero.
            while(this->IsDataAvailable())
            {
                bytesRead += ReadByte(nextChar,
                                      msTimeout);
                dataBuffer.push_back(nextChar);
            }
        }
        else
        {
            // Reserve enough space in the buffer to store the incoming data.
            dataBuffer.reserve(numOfBytes);

            for (unsigned int i=0; i<numOfBytes; ++i)
            {
                // Throw an exception if we are unable to read the current time.            
                if (gettimeofday(&current_time,
                                 NULL) < 0)
                {
                    throw std::runtime_error(strerror(errno));
                }

                // Obtain the elapsed time.
                elapsed_time = current_time - entry_time;

                // Calculate the elapsed number of milliseconds.
                elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                              elapsed_time.tv_usec / MICROSECONDS_PER_MS);

                // If more than msTimeout milliseconds have elapsed while
                // waiting for data, then we throw a ReadTimeout exception.
                if (msTimeout > 0 &&
                    elapsed_ms > msTimeout)
                {
                    throw ReadTimeout();
                }
                
                remaining_ms = msTimeout - elapsed_ms;

                bytesRead += ReadByte(nextChar,
                                      remaining_ms);

                dataBuffer.push_back(nextChar);
            }
        }
        
        return dataBuffer.size();
    }

    inline
    int
    SerialPort::Implementation::ReadByte(unsigned char&      charBuffer,
                                         const unsigned int& msTimeout)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        unsigned int elapsed_ms;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.  
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        pthread_mutex_lock(&mQueueMutex);
        int queueSize = mInputBuffer.size();
        pthread_mutex_unlock(&mQueueMutex);

        // Wait for data to be available.
        while (queueSize == 0)
        {
            // Throw an exception if we are unable to read the current time.  
            if (gettimeofday(&current_time,
                             NULL) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }

            // Obtain the total elapsed time.
            elapsed_time = current_time - entry_time;

            // Calculate the elapsed number of milliseconds.
            elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                          elapsed_time.tv_usec / MICROSECONDS_PER_MS);

            // Throw a ReadTimeout exception if more than msTimeout milliseconds
            // have elapsed while waiting for data.
            if (msTimeout > 0 &&
                elapsed_ms > msTimeout)
            {
                throw ReadTimeout();
            }

            // Sleep for 1ms (1000us) for data to arrive.
            usleep(MICROSECONDS_PER_MS);

            pthread_mutex_lock(&mQueueMutex);
            queueSize = mInputBuffer.size();
            pthread_mutex_unlock(&mQueueMutex);
        }

        // Return the first byte and remove it from the queue.
        pthread_mutex_lock(&mQueueMutex);
        charBuffer = mInputBuffer.front();
        mInputBuffer.pop();
        queueSize = mInputBuffer.size();
        pthread_mutex_unlock(&mQueueMutex);

        return 1;
    }

    inline
    int
    SerialPort::Implementation::ReadLine(std::string&        dataString,
                                         const char&         lineTerminator,
                                         const unsigned int& msTimeout)
    {
        // Clear the data string.
        dataString.clear();

        unsigned char nextChar = 0;
        size_t bytesRead = 0;

        unsigned int elapsed_ms;
        unsigned int remaining_ms;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        const int MICROSECONDS_PER_MS  = 1000;
        const int MILLISECONDS_PER_SEC = 1000;

        do
        {
            // Throw an exception if we are unable to read the current time.            
            if (gettimeofday(&current_time,
                             NULL) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }

            // Obtain the elapsed time.
            elapsed_time = current_time - entry_time;

            // Calculate the elapsed number of milliseconds.
            elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                          elapsed_time.tv_usec / MICROSECONDS_PER_MS);

            // If more than msTimeout milliseconds have elapsed while
            // waiting for data, then we throw a ReadTimeout exception.
            if (msTimeout > 0 &&
                elapsed_ms > msTimeout)
            {
                throw ReadTimeout();
            }

            remaining_ms = msTimeout - elapsed_ms;

            bytesRead += this->ReadByte(nextChar,
                                        remaining_ms);
            dataString += nextChar;
        } while (nextChar != lineTerminator);
        
        return bytesRead;
    }

    inline
    void
    SerialPort::Implementation::WriteByte(const unsigned char& dataByte)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Write the byte to the serial port.
        this->Write(&dataByte,
                    1);
        return;
    }

    inline
    void
    SerialPort::Implementation::Write(const SerialPort::DataBuffer& dataBuffer)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Nothing needs to be done if there is no data in the buffer.
        if (0 == dataBuffer.size())
        {
            return;
        }

        // Allocate memory for storing the contents of the
        // dataBuffer. This allows us to write all the data using a single
        // call to write() instead of writing one byte at a time.
        unsigned char* local_buffer = new unsigned char[dataBuffer.size()];
        
        if (0 == local_buffer)
        {
            throw std::runtime_error(std::string(__FUNCTION__) +
                ": Cannot allocate memory while writing data to the serial port.");
        }

        // Copy the data into local_buffer.
        std::copy(dataBuffer.begin(),
                  dataBuffer.end(),
                  local_buffer);

        // Write data to the serial port.
        try
        {
            this->Write(local_buffer,
                        dataBuffer.size());
        }
        catch( ... )
        {
            // Free the allocated memory.
            delete [] local_buffer;
            throw;
        }

        // Free the allocated memory.
        delete [] local_buffer;
        return;
    }

    inline
    void
    SerialPort::Implementation::Write(const unsigned char* dataBuffer,
                                      const unsigned int&  bufferSize)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        int num_of_bytes_written = -1;
        
        do
        {
            num_of_bytes_written = write(mFileDescriptor,
                                         dataBuffer,
                                         bufferSize);
        }
        while (num_of_bytes_written < 0 &&
               errno == EAGAIN &&
               errno != EWOULDBLOCK);

        if (num_of_bytes_written < 0 ||
            num_of_bytes_written < (int)bufferSize)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetDtr(const bool& dtrState)
    {
        this->SetModemControlLine(TIOCM_DTR, 
                                   dtrState);
        return;
    }

    inline
    bool
    SerialPort::Implementation::GetDtr()
    {
        return this->GetModemControlLine(TIOCM_DTR);
    }    

    inline
    void
    SerialPort::Implementation::SetRts(const bool& rtsState)
    {
        this->SetModemControlLine(TIOCM_RTS, 
                                  rtsState);
        return;
    }

    inline
    bool
    SerialPort::Implementation::GetRts()
    {
        return this->GetModemControlLine(TIOCM_RTS);
    }    

    inline
    bool
    SerialPort::Implementation::GetCts()
    {
        return this->GetModemControlLine(TIOCM_CTS);
    }    

    inline
    bool
    SerialPort::Implementation::GetDsr()
    {
        return this->GetModemControlLine(TIOCM_DSR);
    }    

    inline
    int
    SerialPort::Implementation::GetFileDescriptor()
    {
        return mFileDescriptor;
    }

    inline
    void
    SerialPort::Implementation::HandlePosixSignal(const int& signalNumber)
    {
        // We only want to deal with SIGIO signals here.
        if (SIGIO != signalNumber)
        {
            return;
        }

        // Check if any data is available at the specified file descriptor.
        int num_of_bytes_available = 0;
        
        if (ioctl(mFileDescriptor,
                  FIONREAD,
                  &num_of_bytes_available) < 0)
        {
            // Ignore any errors and return immediately.
            return;
        }

        // Try to get the mutex
        if (pthread_mutex_trylock(&mQueueMutex) == 0)
        {
            // Transfer any pending data from the mShadowInputBuffer
            // into the mInputBuffer.
            while(!mShadowInputBuffer.empty())
            {
                mInputBuffer.push(mShadowInputBuffer.front());
                mShadowInputBuffer.pop();
            }

            // If data is available, read all available data and store
            // it in the corresponding input buffer.
            for(int i=0; i<num_of_bytes_available; ++i)
            {
                unsigned char next_byte;
                
                if (read(mFileDescriptor,
                         &next_byte,
                         1) > 0)
                {
                    mInputBuffer.push(next_byte);
                }
                else
                {
                    pthread_mutex_unlock(&mQueueMutex);
                    break;
                }
            }

            // Release the mutex
            pthread_mutex_unlock(&mQueueMutex);
        }
        else
        {
            // The Mutex was already locked, so all available data will be read
            // from the serial port and stored in the mShadowInputBuffer.
            for (int i=0; i<num_of_bytes_available; ++i)
            {
                unsigned char next_byte;
                
                if (read(mFileDescriptor,
                         &next_byte,
                         1) > 0)
                {
                    mShadowInputBuffer.push(next_byte);
                }
                else
                {
                    break;
                }
            }
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetModemControlLine(const int&  modemLine,
                                                    const bool& lineState)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (modemLine != TIOCM_LE  &&
            modemLine != TIOCM_DTR &&
            modemLine != TIOCM_RTS &&
            modemLine != TIOCM_ST  &&
            modemLine != TIOCM_SR  &&
            modemLine != TIOCM_CTS &&
            modemLine != TIOCM_CAR &&
            modemLine != TIOCM_CD  &&
            modemLine != TIOCM_RNG &&
            modemLine != TIOCM_RI  &&
            modemLine != TIOCM_DSR)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set or unset the specified bit according to the value of lineState.
        int ioctl_result = -1;
        
        if (true == lineState)
        {
            int set_line_mask = modemLine;
            ioctl_result = ioctl(mFileDescriptor, 
                                 TIOCMBIS,
                                 &set_line_mask);
        }
        else
        {
            int reset_line_mask = modemLine;
            ioctl_result = ioctl(mFileDescriptor, 
                                 TIOCMBIC,
                                 &reset_line_mask);
        }

        // Check for errors.
        if (-1 == ioctl_result)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    bool
    SerialPort::Implementation::GetModemControlLine(const int& modemLine)
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Use an ioctl() call to get the state of the line.
        int serial_port_state = 0;
        
        if (-1 == ioctl(mFileDescriptor,
                        TIOCMGET,
                        &serial_port_state))
        {
            throw std::runtime_error(strerror(errno));
        }

        // :TODO: Verify that modemLine is a valid value.
        return (serial_port_state & modemLine);
    }
}

namespace
{
    const timeval
    operator-(const timeval& firstOperand,
              const timeval& secondOperand)
    {
        /**
         * @NOTE: This implementation may result in undefined behavior if the
         *        platform uses unsigned values for storing tv_sec and tv_usec
         *        members of struct timeval.
         */

        timeval result;

        // Take the difference of individual members of the two operands.
        result.tv_sec  = firstOperand.tv_sec - secondOperand.tv_sec;
        result.tv_usec = firstOperand.tv_usec - secondOperand.tv_usec;

        // If abs(result.tv_usec) is larger than MICROSECONDS_PER_SECOND,
        // then increment/decrement result.tv_sec accordingly.
        if (std::abs(result.tv_usec) > LibSerial::MICROSECONDS_PER_SEC)
        {
            int num_of_seconds = (result.tv_usec / LibSerial::MICROSECONDS_PER_SEC);
            result.tv_sec  += num_of_seconds;
            result.tv_usec -= (LibSerial::MICROSECONDS_PER_SEC * num_of_seconds);
        }
        
        return result;
    }
}