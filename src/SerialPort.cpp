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

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

namespace LibSerial 
{
    class SerialPort::Implementation
    {
    public:
        /**
         * @brief Default Constructor.
         */
        Implementation();

        /**
         * @brief Constructor that allows one to create a SerialPort
         *        instance and also initialize the corresponding serial
         *        port with the specified parameters.
         * @param fileName The file descriptor of the serial stream object.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial stream object.
         * @param numberOfStopBits The number of stop bits.
         * @param flowControlType Flow control for the serial data stream.
         */
        Implementation(const std::string&   fileName,
                       const BaudRate&      baudRate,
                       const CharacterSize& characterSize,
                       const FlowControl&   flowControlType,
                       const Parity&        parityType,
                       const StopBits&      stopBits);

        /**
         * @brief Default Destructor.
         */
        ~Implementation();

        /**
         * @brief Opens the serial port associated with the specified
         *        fileName, and the specified mode, openMode.
         * @param fileName The file descriptor of the serial stream object.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& filename,
                  std::ios_base::openmode openMode);

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Flushes the serial port input buffer.
         */
        void FlushInputBuffer();

        /**
         * @brief Flushes the serial port output buffer.
         */
        void FlushOutputBuffer();

        /**
         * @brief Flushes the serial port input and output buffers.
         */
        void FlushIOBuffers();

        /**
         * @brief Determines if data is available at the serial port.
         */
        bool IsDataAvailable();

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen();

        /**
         * @brief Sets all serial port paramters to their default values.
         */
        void SetDefaultSerialPortParameters();

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
        void SetVMin(const short vmin);

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
        void SetVTime(const short vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

        /**
         * @brief Sets the serial port DTR line status.
         * @param dtrState The state to set the DTR line
         */
        void SetDTR(const bool dtrState);

        /**
         * @brief Gets the serial port DTR line status.
         */
        bool GetDTR();

        /**
         * @brief Sets the serial port RTS line status.
         * @param dtrState The state to set the RTS line
         */
        void SetRTS(const bool rtsState);

        /**
         * @brief Gets the serial port RTS line status.
         */
        bool GetRTS();

        /**
         * @brief Gets the serial port CTS line status.
         */
        bool GetCTS();

        /**
         * @brief Gets the serial port DSR line status.
         */
        bool GetDSR();

        /**
         * @brief Gets the serial port file descriptor.
         */
        int GetFileDescriptor();

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numberOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataBuffer on
         *        return from this method.
         * @param charBuffer The character buffer to place serial data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(unsigned char&     charBuffer,
                  const unsigned int numberOfBytes = 0,
                  const unsigned int msTimeout = 0);

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numberOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataBuffer on
         *        return from this method.
         * @param dataBuffer The data buffer to place serial data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(SerialPort::DataBuffer& dataBuffer,
                  const unsigned int      numberOfBytes = 0,
                  const unsigned int      msTimeout = 0);

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numberOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataBuffer on
         *        return from this method.
         * @param dataString The data string read from the serial port.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(std::string&       dataString,
                  const unsigned int numberOfBytes = 0,
                  const unsigned int msTimeout  = 0);

        /**
         * @brief Reads a single byte from the serial port.
         *        If no data is available within the specified number
         *        of milliseconds (msTimeout), then this method will
         *        throw a ReadTimeout exception. If msTimeout is 0,
         *        then this method will block until data is available.
         * @param msTimeout The timeout period in milliseconds.
         */
        void ReadByte(unsigned char&     charBuffer, 
                      const unsigned int msTimeout = 0);

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
         */
        void ReadLine(std::string&       dataString,
                      const char         lineTerminator = '\n',
                      const unsigned int msTimeout = 0);

         /**
         * @brief Writes a character array buffer to the serial port.
         * @param charBuffer The character array to be written to the serial port.
         * @param numberOfBytes The number of bytes to be written to the serial port.
         */
        void Write(const unsigned char* charBuffer,
                   const unsigned int   numberOfBytes);

        /**
         * @brief Writes a DataBuffer vector to the serial port.
         * @param dataBuffer The DataBuffer vector to be written to the serial port.
         */
        void Write(const SerialPort::DataBuffer& dataBuffer);

        /**
         * @brief Writes a std::string to the serial port.
         * @param dataString The std::string to be written to the serial port.
         */
        void Write(const std::string& dataString);

        /**
         * @brief Writes a single byte to the serial port.
         * @param charbuffer The byte to be written to the serial port.
         */
        void WriteByte(const unsigned char charbuffer);

    private:

        /**
         * @brief Set the specified modem control line to the specified value.
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @param lineState State of the modem line after successful
         *        call to this method.
         */
        void SetModemControlLine(const int modemLine,
                                 const bool lineState);

        /**
         * @brief Get the current state of the specified modem control line.
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @return True if the specified line is currently set and false
         *         otherwise.
         */
        bool GetModemControlLine(const int modemLine);

        /**
         * @brief Sets the current state of the serial port blocking status.
         * @param blockingStatus The serial port blocking status to be set,
         *        true if to be set blocking, false if to be set non-blocking.
         */
        void SetPortBlockingStatus(const bool blockingStatus);

        /**
         * @brief Gets the current state of the serial port blocking status.
         * @return True if port is blocking, false if port non-blocking.
         */
        bool GetPortBlockingStatus();

        /**
         * @brief Sets the default Linux specific line discipline modes.
         */
        void SetDefaultLinuxSpecificModes();

        /**
         * @brief Sets the default serial port input modes.
         */
        void SetDefaultInputModes();

        /**
         * @brief Sets the default serial port output modes.
         */
        void SetDefaultOutputModes();

        /**
         * @brief Sets the default serial port control modes.
         */
        void SetDefaultControlModes();

        /**
         * @brief Sets the default serial port local modes.
         */
        void SetDefaultLocalModes();

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
    };

    SerialPort::SerialPort()
        : mImpl(new Implementation())
    {
        /* Empty */
    }

    SerialPort::SerialPort(const std::string&   fileName,
                           const BaudRate&      baudRate,
                           const CharacterSize& characterSize,
                           const FlowControl&   flowControlType,
                           const Parity&        parityType,
                           const StopBits&      stopBits)
        : mImpl(new Implementation(fileName,
                                   baudRate,
                                   characterSize,
                                   flowControlType,
                                   parityType,
                                   stopBits))
    {
        /* Empty */
    }

    SerialPort::~SerialPort()
    {
        // Close the serial port if it is open.
        if (this->IsOpen())
        {
            this->Close();
        }

        return;
    }

    void
    SerialPort::Open(const std::string& filename,
                     std::ios_base::openmode openMode)
    {
        mImpl->Open(filename,
                    openMode);
        return;
    }

    void
    SerialPort::Close()
    {
        mImpl->Close();
        return;
    }

    void
    SerialPort::FlushInputBuffer()
    {
        mImpl->FlushInputBuffer();
        return;
    }

    void
    SerialPort::FlushOutputBuffer()
    {
        mImpl->FlushOutputBuffer();
        return;
    }

    void
    SerialPort::FlushIOBuffers()
    {
        mImpl->FlushIOBuffers();
        return;
    }

    bool
    SerialPort::IsDataAvailable()
    {
        return mImpl->IsDataAvailable();
    }

    bool
    SerialPort::IsOpen()
    {
        return mImpl->IsOpen();
    }

    void
    SerialPort::SetDefaultSerialPortParameters()
    {
        mImpl->SetDefaultSerialPortParameters();
        return;
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
        mImpl->SetParity(parityType);
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
    SerialPort::SetVMin(const short vmin)
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
    SerialPort::SetVTime(const short vtime)
    {
        mImpl->SetVTime(vtime);
        return;
    }

    short
    SerialPort::GetVTime()
    {
        return mImpl->GetVTime();
    }

    void
    SerialPort::SetDTR(const bool dtrState)
    {
        mImpl->SetDTR(dtrState);
        return;
    }

    bool
    SerialPort::GetDTR()
    {
        return mImpl->GetDTR();
    }

    void
    SerialPort::SetRTS(const bool rtsState)
    {
        mImpl->SetRTS(rtsState);
        return;
    }

    bool
    SerialPort::GetRTS()
    {
        return mImpl->GetRTS();
    }

    bool
    SerialPort::GetCTS()
    {
        return mImpl->GetCTS();
    }

    bool
    SerialPort::GetDSR() 
    {
        return mImpl->GetDSR();
    }

    int
    SerialPort::GetFileDescriptor()
    {
        return mImpl->GetFileDescriptor();
    }

    void
    SerialPort::Read(unsigned char&     charBuffer,
                     const unsigned int numberOfBytes,
                     const unsigned int msTimeout)
    {
        mImpl->Read(charBuffer,
                    numberOfBytes,
                    msTimeout);
        return;
    }

    void
    SerialPort::Read(SerialPort::DataBuffer& dataBuffer,
                     const unsigned int      numberOfBytes,
                     const unsigned int      msTimeout)
    {
        mImpl->Read(dataBuffer,
                    numberOfBytes,
                    msTimeout);
        return;
    }

    void
    SerialPort::Read(std::string&       dataString,
                     const unsigned int numberOfBytes,
                     const unsigned int msTimeout)
    {
        mImpl->Read(dataString,
                    numberOfBytes,
                    msTimeout);
        return;
    }

    void
    SerialPort::ReadByte(unsigned char&     charBuffer,
                         const unsigned int msTimeout)
    {
        mImpl->ReadByte(charBuffer,
                        msTimeout);
        return;
    }

    void
    SerialPort::ReadLine(std::string&       dataString,
                         const char         lineTerminator,
                         const unsigned int msTimeout)
    {
        mImpl->ReadLine(dataString,
                        lineTerminator,
                        msTimeout);
        return;
    }

    void
    SerialPort::Write(const unsigned char* charBuffer,
                      const unsigned int   numberOfBytes)
    {
        mImpl->Write(charBuffer,
                     numberOfBytes);
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
        mImpl->Write(dataString);
        return;
    }

    void
    SerialPort::WriteByte(const unsigned char charBuffer)
    {
        mImpl->WriteByte(charBuffer);
        return;
    }


    /** ------------------------------------------------------------ */
    inline
    SerialPort::Implementation::Implementation()
        : mFileDescriptor(-1)
    {
        /* empty */
    }

    inline
    SerialPort::Implementation::Implementation(const std::string&   fileName,
                                               const BaudRate&      baudRate,
                                               const CharacterSize& characterSize,
                                               const FlowControl&   flowControlType,
                                               const Parity&        parityType,
                                               const StopBits&      stopBits)
        : mFileDescriptor(-1)
    {
        this->Open(fileName, std::ios_base::in | std::ios_base::out);
        this->SetBaudRate(baudRate);
        this->SetCharacterSize(characterSize);
        this->SetFlowControl(flowControlType);
        this->SetParity(parityType);
        this->SetNumberOfStopBits(stopBits);
        return;
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
    SerialPort::Implementation::Open(const std::string& filename,
                                     std::ios_base::openmode openMode)
    {
        // Throw an exception if the port is already open.
        if (this->IsOpen())
        {
            throw AlreadyOpen(ERR_MSG_PORT_ALREADY_OPEN);
        }

        // We only allow three different combinations of ios_base::openmode so we can
        // use a switch here to construct the flags to be used with the open() system call.
        // Since we are dealing with the serial port we need to use the O_NOCTTY option.
        int flags = 0;
        
        if (openMode == (std::ios_base::in | std::ios_base::out))
        {
            flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);
        } 
        else if (openMode == std::ios_base::in)
        {
            flags = (O_RDONLY | O_NOCTTY | O_NONBLOCK);
        } 
        else if (openMode == std::ios_base::out)
        {
            flags = (O_WRONLY | O_NOCTTY | O_NONBLOCK);
        } 
        else 
        {
            return;
        }

        // Try to open the serial port. 
        mFileDescriptor = open(filename.c_str(), flags);
        
        if (this->mFileDescriptor < 0)
        {
            close(this->mFileDescriptor);
            throw OpenFailed(strerror(errno));
        }

        // Save the current settings of the serial port so they can be
        // restored when the serial port is closed.
        if (tcgetattr(this->mFileDescriptor,
                      &mOldPortSettings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Set up the default configuration for the serial port.
        this->SetDefaultSerialPortParameters();

        // Flush the input and output buffers associated with the port.
        this->FlushIOBuffers();

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

        // Restore the old settings of the port.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &mOldPortSettings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Otherwise, close the serial port and set the file descriptor
        // to an invalid value.
        if (close(this->mFileDescriptor) < 0) 
        {
            throw std::runtime_error(strerror(errno));
        } 

        // Set the file descriptor to an invalid value, -1. 
        mFileDescriptor = -1;
        return;
    }

    inline
    void
    SerialPort::Implementation::FlushInputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCIFLUSH) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::FlushOutputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCOFLUSH) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::FlushIOBuffers()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCIOFLUSH) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    bool
    SerialPort::Implementation::IsOpen()
    {
        return (-1 != this->mFileDescriptor);
    }

    inline
    bool
    SerialPort::Implementation::IsDataAvailable()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        int number_of_bytes_available = 0;
        bool dataAvailableStatus = false;

        int result = ioctl(this->mFileDescriptor,
                           FIONREAD,
                           &number_of_bytes_available);
        
        if (result >= 0 &&
            number_of_bytes_available > 0)
        {
            dataAvailableStatus = true;
        }

        return dataAvailableStatus;
    }

    inline
    void 
    SerialPort::Implementation::SetDefaultSerialPortParameters()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        #ifdef __linux__
            SetDefaultLinuxSpecificModes();
        #endif

        SetDefaultInputModes();
        SetDefaultOutputModes();
        SetDefaultControlModes();
        SetDefaultLocalModes();

        SetBaudRate(BaudRate::BAUD_DEFAULT);
        SetCharacterSize(CharacterSize::CHAR_SIZE_DEFAULT);
        SetFlowControl(FlowControl::FLOW_CONTROL_DEFAULT);
        SetParity(Parity::PARITY_DEFAULT);
        SetNumberOfStopBits(StopBits::STOP_BITS_DEFAULT);
        SetVMin(VMIN_DEFAULT);
        SetVTime(VTIME_DEFAULT);

        return;
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
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the baud rate for both input and output.
        if (cfsetspeed(&port_settings, (speed_t)baudRate) < 0 ||
            cfsetospeed(&port_settings, (speed_t)baudRate) < 0)
        {
            // If applying the baud rate settings fail, throw an exception.
            throw std::runtime_error(ERR_MSG_INVALID_BAUD_RATE);
        }

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            // If applying the settings fails, throw an exception.
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    BaudRate
    SerialPort::Implementation::GetBaudRate()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Read the input and output baud rates.
        speed_t input_baud = cfgetispeed(&port_settings);
        speed_t output_baud = cfgetospeed(&port_settings);

        // Make sure that the input and output baud rates are
        // equal. Otherwise, we do not know which one to return.
        if (input_baud != output_baud)
        {
            throw std::runtime_error(ERR_MSG_INVALID_BAUD_RATE);
            return BaudRate::BAUD_INVALID;
        }

        // Obtain the input baud rate from the current settings.
        return BaudRate(input_baud);
    }

    inline
    void
    SerialPort::Implementation::SetCharacterSize(const CharacterSize& characterSize)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(ERR_MSG_INVALID_CHARACTER_SIZE);
        }

        // Set the character size to the specified value. If the character
        // size is not 8 then it is also important to set ISTRIP. Setting
        // ISTRIP causes all but the 7 low-order bits to be set to
        // zero. Otherwise they are set to unspecified values and may
        // cause problems. At the same time, we should clear the ISTRIP
        // flag when the character size is 8 otherwise the MSB will always
        // be set to zero (ISTRIP does not check the character size
        // setting; it just sets every bit above the low 7 bits to zero).
        if (characterSize == CharacterSize::CHAR_SIZE_8)
        {
            port_settings.c_iflag &= ~ISTRIP; // Clear the ISTRIP flag.
        }
        else
        {
            port_settings.c_iflag |= ISTRIP;  // Set the ISTRIP flag.
        }

        // Set the character size.
        port_settings.c_cflag &= ~CSIZE;                               // Clear all CSIZE bits.
        port_settings.c_cflag |= static_cast<tcflag_t>(characterSize); // Set the character size.

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    CharacterSize
    SerialPort::Implementation::GetCharacterSize()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Read the character size from the setttings.
        return CharacterSize(port_settings.c_cflag & CSIZE);
    }

    inline
    void
    SerialPort::Implementation::SetFlowControl(const FlowControl& flowControlType)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Flush the input and output buffers associated with the port.
        if (tcflush(this->mFileDescriptor,
                    TCIOFLUSH) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(ERR_MSG_INVALID_FLOW_CONTROL);
        }

        // Set the flow control. Hardware flow control uses the RTS (Ready
        // To Send) and CTS (clear to Send) lines. Software flow control
        // uses IXON|IXOFF
        switch(flowControlType)
        {
        case FlowControl::FLOW_CONTROL_HARDWARE:
            port_settings.c_iflag &= ~ (IXON|IXOFF);
            port_settings.c_cflag |= CRTSCTS;
            port_settings.c_cc[VSTART] = _POSIX_VDISABLE;
            port_settings.c_cc[VSTOP] = _POSIX_VDISABLE;
            break;
        case FlowControl::FLOW_CONTROL_SOFTWARE:
            port_settings.c_iflag |= IXON|IXOFF;
            port_settings.c_cflag &= ~CRTSCTS;
            port_settings.c_cc[VSTART] = CTRL_Q; // 0x11 (021) ^q
            port_settings.c_cc[VSTOP]  = CTRL_S; // 0x13 (023) ^s
            break;
        case FlowControl::FLOW_CONTROL_NONE:
            port_settings.c_iflag &= ~(IXON|IXOFF);
            port_settings.c_cflag &= ~CRTSCTS;
            break;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_FLOW_CONTROL);
            break;
        }
        
        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    FlowControl
    SerialPort::Implementation::GetFlowControl()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Check if IXON and IXOFF are set in c_iflag. If both are set and
        // VSTART and VSTOP are set to 0x11 (^Q) and 0x13 (^S) respectively,
        // then we are using software flow control.
        if (port_settings.c_iflag & IXON &&
            port_settings.c_iflag & IXOFF &&
            CTRL_Q == port_settings.c_cc[VSTART] &&
            CTRL_S == port_settings.c_cc[VSTOP])
        {
            return FlowControl::FLOW_CONTROL_SOFTWARE;
        }
        else if (!(port_settings.c_iflag & IXON ||
                   port_settings.c_iflag & IXOFF))
        {
            if (port_settings.c_cflag & CRTSCTS)
            {
                // If neither IXON or IXOFF is set then we must have hardware flow
                // control.
                return FlowControl::FLOW_CONTROL_HARDWARE;
            }
            else
            {
                return FlowControl::FLOW_CONTROL_NONE;
            }
        }

        // If none of the above conditions are satisfied then the serial port
        // is using a flow control setup which we do not support at present.
        return FlowControl::FLOW_CONTROL_INVALID;
    }

    inline
    void
    SerialPort::Implementation::SetParity(const Parity& parityType)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the parity type
        switch(parityType)
        {
        case Parity::PARITY_EVEN:
            port_settings.c_cflag |= PARENB;
            port_settings.c_cflag &= ~PARODD;
            port_settings.c_iflag |= INPCK;
            break;
        case Parity::PARITY_ODD:
            port_settings.c_cflag |= PARENB;
            port_settings.c_cflag |= PARODD;
            port_settings.c_iflag |= INPCK;
            break;
        case Parity::PARITY_NONE:
            port_settings.c_cflag &= ~PARENB;
            port_settings.c_iflag |= IGNPAR;
            break;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_PARITY);
            break;
        }

        // Apply the modified port settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    Parity
    SerialPort::Implementation::GetParity()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Get the parity setting from the termios structure. 
        if (port_settings.c_cflag & PARENB)
        {
            // parity is enabled.
            if (port_settings.c_cflag & PARODD)
            {
                return Parity::PARITY_ODD; // odd parity
            }
            else
            {
                return Parity::PARITY_EVEN; // even parity
            }
        }
        else
        {
            return Parity::PARITY_NONE; // no parity.
        }

        return Parity::PARITY_INVALID; // execution should never reach here.
    }

    inline
    void
    SerialPort::Implementation::SetNumberOfStopBits(const StopBits& numberOfStopBits)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Set the number of stop bits.
        switch(numberOfStopBits)
        {
        case StopBits::STOP_BITS_1:
            port_settings.c_cflag &= ~CSTOPB;
            break;
        case StopBits::STOP_BITS_2:
            port_settings.c_cflag |= CSTOPB;
            break;
        default: 
            throw std::invalid_argument(ERR_MSG_INVALID_STOP_BITS);
            break;
        }

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    StopBits
    SerialPort::Implementation::GetNumberOfStopBits()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
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
        else
        {
            return StopBits::STOP_BITS_1;
        }
    }

    inline
    void 
    SerialPort::Implementation::SetVMin(const short vmin)
    {
        // Throw an exception if the serial port is not open.
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
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_cc[VMIN] = (cc_t)vmin;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    short 
    SerialPort::Implementation::GetVMin()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return port_settings.c_cc[VMIN];
    }

    inline
    void 
    SerialPort::Implementation::SetVTime(const short vtime)
    {
        // Throw an exception if the serial port is not open.
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
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_cc[VTIME] = (cc_t)vtime;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    short 
    SerialPort::Implementation::GetVTime() 
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return port_settings.c_cc[VTIME];
    }

    inline
    void
    SerialPort::Implementation::SetDTR(const bool dtrState)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        this->SetModemControlLine(TIOCM_DTR, 
                                   dtrState);
        return;
    }

    inline
    bool
    SerialPort::Implementation::GetDTR()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->GetModemControlLine(TIOCM_DTR);
    }    

    inline
    void
    SerialPort::Implementation::SetRTS(const bool rtsState)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        this->SetModemControlLine(TIOCM_RTS, 
                                  rtsState);
        return;
    }

    inline
    bool
    SerialPort::Implementation::GetRTS()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->GetModemControlLine(TIOCM_RTS);
    }    

    inline
    bool
    SerialPort::Implementation::GetCTS()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->GetModemControlLine(TIOCM_CTS);
    }    

    inline
    bool
    SerialPort::Implementation::GetDSR()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->GetModemControlLine(TIOCM_DSR);
    }    

    inline
    int
    SerialPort::Implementation::GetFileDescriptor()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->mFileDescriptor;
    }

    inline
    void
    SerialPort::Implementation::SetModemControlLine(const int  modemLine,
                                                    const bool lineState)
    {
        // Throw an exception if the serial port is not open.
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
            throw std::invalid_argument(strerror(errno));
        }

        // Set or unset the specified bit according to the value of lineState.
        int ioctl_result = -1;
        
        if (true == lineState)
        {
            int set_line_mask = modemLine;
            ioctl_result = ioctl(this->mFileDescriptor, 
                                 TIOCMBIS,
                                 &set_line_mask);
        }
        else
        {
            int reset_line_mask = modemLine;
            ioctl_result = ioctl(this->mFileDescriptor, 
                                 TIOCMBIC,
                                 &reset_line_mask);
        }

        // Check for errors.
        if (ioctl_result  < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    bool
    SerialPort::Implementation::GetModemControlLine(const int modemLine)
    {
        // Throw an exception if the serial port is not open.
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
            throw std::invalid_argument(strerror(errno));
        }
        
        // Use an ioctl() call to get the state of the line.
        int serial_port_state = 0;
        
        if (ioctl(this->mFileDescriptor,
                  TIOCMGET,
                  &serial_port_state) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        return (serial_port_state & modemLine);
    }

    inline
    void
    SerialPort::Implementation::SetPortBlockingStatus(const bool blockingStatus)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        int flags = fcntl(this->mFileDescriptor, F_GETFL, 0);
        
        if (blockingStatus == true)
        {
            if (fcntl(this->mFileDescriptor, 
                      F_SETFL, 
                      flags &~ O_NONBLOCK) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }
        }
        else
        {
            if (fcntl(this->mFileDescriptor, 
                      F_SETFL, 
                      flags | O_NONBLOCK) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }
        }
    }

    inline
    bool
    SerialPort::Implementation::GetPortBlockingStatus()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        bool blocking_status = false;

        int flags = fcntl(this->mFileDescriptor, F_GETFL, 0);
        
        if (flags == (flags | O_NONBLOCK))
        {
            blocking_status = true;
        }

        return blocking_status;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultLinuxSpecificModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // @NOTE - termios.c_line is not a standard element of the termios
        // structure, (as per the Single Unix Specification 3).
        port_settings.c_line = '\0';

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultInputModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Ignore Break conditions on input.
        port_settings.c_iflag = IGNBRK;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultOutputModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_oflag = 0;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultControlModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Enable the receiver (CREAD) and ignore modem control lines (CLOCAL).
        port_settings.c_cflag |= CREAD | CLOCAL;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultLocalModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        port_settings.c_lflag = 0;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::Read(unsigned char&     charBuffer,
                                     const unsigned int numberOfBytes,
                                     const unsigned int msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (numberOfBytes == 0)
        {
            return;
        }

        unsigned int elapsed_ms = 0;
        
        int number_of_bytes_read = 0;
        int read_result = 0;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        // Loop until the number of bytes requested have been read or the timeout has elapsed.
        while (number_of_bytes_read < (int)numberOfBytes)
        {
            read_result = read(this->mFileDescriptor,
                               &charBuffer + number_of_bytes_read,
                               numberOfBytes - number_of_bytes_read);
            
            if (read_result > 0)
            {
                number_of_bytes_read += read_result;

                if (number_of_bytes_read == (int)numberOfBytes)
                {
                    break;
                }
            }
            else if (read_result <= 0 &&
                     errno != EWOULDBLOCK)
            {
                throw std::runtime_error(strerror(errno));
            }
            
            // Throw an exception if we are unable to read the current time.
            if (gettimeofday(&current_time,
                             NULL) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }

            // Obtain the total elapsed time.
            timersub(&current_time, &entry_time, &elapsed_time);

            // Calculate the elapsed number of milliseconds.
            elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                          elapsed_time.tv_usec / MICROSECONDS_PER_MS);

            // Throw a ReadTimeout exception if more than msTimeout milliseconds
            // have elapsed while waiting for data.
            if (msTimeout > 0 &&
                elapsed_ms > msTimeout)
            {
                throw ReadTimeout(ERR_MSG_READ_TIMEOUT);
            }

            // Sleep for 1ms (1000us) for data to arrive.
            usleep(1000);
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::Read(SerialPort::DataBuffer& dataBuffer,
                                     const unsigned int      numberOfBytes,
                                     const unsigned int      msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Clear the data buffer.
        dataBuffer.resize(0);
        unsigned char next_char = 0;

        unsigned int elapsed_ms = 0;
        unsigned int remaining_ms = 0;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        if (numberOfBytes == 0)
        {
            // Read all available data if numberOfBytes is zero.
            while(this->IsDataAvailable())
            {
                this->Read(next_char,
                           1,
                           remaining_ms);

                dataBuffer.push_back(next_char);
            }
        }
        else
        {
            // Reserve enough space in the buffer to store the incoming data.
            dataBuffer.reserve(numberOfBytes);

            for (unsigned int i=0; i<numberOfBytes; ++i)
            {
                // Throw an exception if we are unable to read the current time.
                if (gettimeofday(&current_time,
                                 NULL) < 0)
                {
                    throw std::runtime_error(strerror(errno));
                }

                // Obtain the elapsed time.
                timersub(&current_time, &entry_time, &elapsed_time);

                // Calculate the elapsed number of milliseconds.
                elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                              elapsed_time.tv_usec / MICROSECONDS_PER_MS);

                // If more than msTimeout milliseconds have elapsed while
                // waiting for data, then we throw a ReadTimeout exception.
                if (msTimeout > 0 &&
                    elapsed_ms > msTimeout)
                {
                    throw ReadTimeout(ERR_MSG_READ_TIMEOUT);
                }
                
                remaining_ms = msTimeout - elapsed_ms;

                this->Read(next_char,
                           1,
                           remaining_ms);

                dataBuffer.push_back(next_char);
            }
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::Read(std::string&       dataString,
                                     const unsigned int numberOfBytes,
                                     const unsigned int msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Clear the data string.
        dataString.clear();

        unsigned char next_char = 0;

        unsigned int elapsed_ms = 0;
        unsigned int remaining_ms = 0;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        if (numberOfBytes == 0)
        {
            // Read all available data if numberOfBytes is zero.
            while(this->IsDataAvailable())
            {
                this->Read(next_char,
                           1,
                           remaining_ms);

                dataString += next_char;
            }
        }
        else
        {
            for (unsigned int i=0; i<numberOfBytes; ++i)
            {
                // Throw an exception if we are unable to read the current time.
                if (gettimeofday(&current_time,
                                 NULL) < 0)
                {
                    throw std::runtime_error(strerror(errno));
                }

                // Obtain the elapsed time.
                timersub(&current_time, &entry_time, &elapsed_time);

                // Calculate the elapsed number of milliseconds.
                elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                              elapsed_time.tv_usec / MICROSECONDS_PER_MS);

                // If more than msTimeout milliseconds have elapsed while
                // waiting for data, then we throw a ReadTimeout exception.
                if (msTimeout > 0 &&
                    elapsed_ms > msTimeout)
                {
                    throw ReadTimeout(ERR_MSG_READ_TIMEOUT);
                }

                remaining_ms = msTimeout - elapsed_ms;

                this->Read(next_char,
                           1,
                           remaining_ms);

                dataString += next_char;
            }
        }
        
        return;
    }

    inline
    void
    SerialPort::Implementation::ReadByte(unsigned char&     charBuffer,
                                         const unsigned int msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        this->Read(charBuffer,
                   1,
                   msTimeout);
        return;
    }

    inline
    void
    SerialPort::Implementation::ReadLine(std::string&       dataString,
                                         const char         lineTerminator,
                                         const unsigned int msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }
        
        // Clear the data string.
        dataString.clear();

        unsigned char next_char = 0;

        unsigned int elapsed_ms = 0;
        unsigned int remaining_ms = 0;

        timeval entry_time;
        timeval current_time;
        timeval elapsed_time;
        
        // Throw an exception if we are unable to read the current time.
        if (gettimeofday(&entry_time,
                         NULL) < 0)
        {
            throw std::runtime_error(strerror(errno));
        }

        while (next_char != lineTerminator)
        {
            // Throw an exception if we are unable to read the current time.
            if (gettimeofday(&current_time,
                             NULL) < 0)
            {
                throw std::runtime_error(strerror(errno));
            }

            // Obtain the elapsed time.
            timersub(&current_time, &entry_time, &elapsed_time);

            // Calculate the elapsed number of milliseconds.
            elapsed_ms = (elapsed_time.tv_sec  * MILLISECONDS_PER_SEC +
                          elapsed_time.tv_usec / MICROSECONDS_PER_MS);

            // If more than msTimeout milliseconds have elapsed while
            // waiting for data, then we throw a ReadTimeout exception.
            if (msTimeout > 0 &&
                elapsed_ms > msTimeout)
            {
                throw ReadTimeout(ERR_MSG_READ_TIMEOUT);
            }

            remaining_ms = msTimeout - elapsed_ms;

            this->Read(next_char,
                       1,
                       remaining_ms);

            dataString += next_char;
        }
        
        return;
    }

    inline
    void
    SerialPort::Implementation::Write(const unsigned char* charBuffer,
                                      const unsigned int   numberOfBytes)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Nothing needs to be done if there is no data in the buffer.
        if (numberOfBytes <= 0)
        {
            return;
        }

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        int num_of_bytes_written = -1;
        
        do
        {
            num_of_bytes_written = write(this->mFileDescriptor,
                                         charBuffer,
                                         numberOfBytes);
        }
        while (num_of_bytes_written <= 0 &&
               errno == EAGAIN &&
               errno != EWOULDBLOCK);

        if (num_of_bytes_written < 0 ||
            num_of_bytes_written < (int)numberOfBytes)
        {
            throw std::runtime_error(strerror(errno));
        }

        return;
    }

    inline
    void
    SerialPort::Implementation::Write(const SerialPort::DataBuffer& dataBuffer)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        std::string data_string(dataBuffer.begin(),
                                dataBuffer.end());

        this->Write(data_string);
        
        return;
    }

    inline
    void
    SerialPort::Implementation::Write(const std::string& dataString)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        this->Write(reinterpret_cast<const unsigned char*>(dataString.c_str()),
                    dataString.length());
        return;
    }

    inline
    void
    SerialPort::Implementation::WriteByte(const unsigned char charBuffer)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        this->Write(&charBuffer,
                    1);

        return;
    }
}