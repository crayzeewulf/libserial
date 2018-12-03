/******************************************************************************
 * @file SerialPort.cpp                                                       *
 * @copyright (C) 2004-2018 LibSerial Development Team. All rights reserved.  *
 * crayzeewulf@gmail.com                                                      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright          *
 *    notice, this list of conditions and the following disclaimer.           *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 *    notice, this list of conditions and the following disclaimer in         *
 *    the documentation and/or other materials provided with the              *
 *    distribution.                                                           *
 * 3. Neither the name PX4 nor the names of its contributors may be           *
 *    used to endorse or promote products derived from this software          *
 *    without specific prior written permission.                              *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS          *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE             *
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,        *
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,       *
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS      *
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED         *
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN          *
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            *
 * POSSIBILITY OF SUCH DAMAGE.                                                *
 *****************************************************************************/

#include "libserial/SerialPort.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <type_traits>
#include <unistd.h>

namespace LibSerial 
{
    /**
     * @brief SerialPort::Implementation is the SerialPort implementation class.
     */
    class SerialPort::Implementation
    {
    public:
        /**
         * @brief Default Constructor.
         */
        Implementation() = default ;

        /**
         * @brief Constructor that allows a SerialPort instance to be 
         *        created and opened, initializing the corresponding
         *        serial port with the specified parameters.
         * @param fileName The file name of the serial port.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial port.
         * @param stopBits The number of stop bits for the serial port.
         * @param flowControlType The flow control type for the serial port.
         */
        Implementation(const std::string&   fileName,
                       const BaudRate&      baudRate,
                       const CharacterSize& characterSize,
                       const FlowControl&   flowControlType,
                       const Parity&        parityType,
                       const StopBits&      stopBits) ;

        /**
         * @brief Default Destructor for a SerialPort object. Closes the
         *        serial port associated with mFileDescriptor if open.
         */
        ~Implementation() ;

        /**
         * @brief Copy construction is disallowed.
         */
        Implementation(const Implementation& otherImplementation) = delete ;

        /**
         * @brief Move construction is disallowed.
         */
        Implementation(const Implementation&& otherImplementation) = delete ;

        /**
         * @brief Copy assignment is disallowed.
         */
        Implementation& operator=(const Implementation& otherImplementation) = delete ;

        /**
         * @brief Move assignment is disallowed.
         */
        Implementation& operator=(const Implementation&& otherImplementation) = delete ;

        /**
         * @brief Opens the serial port associated with the specified
         *        file name and the specified mode.
         * @param fileName The file name of the serial port.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName,
                  const std::ios_base::openmode& openMode) ;

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close() ;

        /**
         * @brief Waits until the write buffer is drained and then returns.
         */
        void DrainWriteBuffer() ;

        /**
         * @brief Flushes the serial port input buffer.
         */
        void FlushInputBuffer() ;

        /**
         * @brief Flushes the serial port output buffer.
         */
        void FlushOutputBuffer() ;

        /**
         * @brief Flushes the serial port input and output buffers.
         */
        void FlushIOBuffers() ;

        /**
         * @brief Determines if data is available at the serial port.
         */
        bool IsDataAvailable() ;

        /**
         * @brief Determines if the serial port is open for I/O.
         * @return Returns true iff the serial port is open.
         */
        bool IsOpen() const ;

        /**
         * @brief Sets all serial port paramters to their default values.
         */
        void SetDefaultSerialPortParameters() ;

        /**
         * @brief Sets the baud rate for the serial port to the specified value
         * @param baudRate The baud rate to be set for the serial port.
         */
        void SetBaudRate(const BaudRate& baudRate) ;

        /**
         * @brief Gets the current baud rate for the serial port.
         * @return Returns the baud rate.
         */
        BaudRate GetBaudRate() const ;

        /**
         * @brief Sets the character size for the serial port.
         * @param characterSize The character size to be set.
         */
        void SetCharacterSize(const CharacterSize& characterSize) ;

        /**
         * @brief Gets the character size being used for serial communication.
         * @return Returns the current character size. 
         */
        CharacterSize GetCharacterSize() const ;

        /**
         * @brief Sets flow control for the serial port.
         * @param flowControlType The flow control type to be set.
         */
        void SetFlowControl(const FlowControl& flowControlType) ;

        /**
         * @brief Get the current flow control setting.
         * @return Returns the flow control type of the serial port.
         */
        FlowControl GetFlowControl() const ;

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity& parityType) ;

        /**
         * @brief Gets the parity type for the serial port.
         * @return Returns the parity type.
         */
        Parity GetParity() const ;

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param stopBits The number of stop bits to set.
         */
        void SetStopBits(const StopBits& stopBits) ;

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetStopBits() const ;

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @param vmin the number of minimum characters to be set.
         */
        void SetVMin(short vmin) ;

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for non-canonical reads.
         */
        short GetVMin() const ;

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         */
        void SetVTime(short vtime) ;

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime() const ;

        /**
         * @brief Sets the serial port DTR line status.
         * @param dtrState The state to set the DTR line
         */
        void SetDTR(bool dtrState) ;

        /**
         * @brief Gets the serial port DTR line status.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDTR() ;

        /**
         * @brief Sets the serial port RTS line status.
         * @param rtsState The state to set the RTS line
         */
        void SetRTS(bool rtsState) ;

        /**
         * @brief Gets the serial port RTS line status.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS() ;

        /**
         * @brief Gets the serial port CTS line status.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCTS() ;

        /**
         * @brief Gets the serial port DSR line status.
         * @return Returns true iff the status of the DSR line is high.
         */
        bool GetDSR() ;

        /**
         * @brief Gets the serial port file descriptor.
         * @return Returns the serial port file descriptor.
         */
        int GetFileDescriptor() const ;

        /**
         * @brief Gets the number of bytes available in the read buffer.
         * @return Returns the number of bytes avilable in the read buffer.
         */
        int GetNumberOfBytesAvailable() ;

        /**
         * @brief Gets a list of available serial ports.
         * @return Returns a std::vector of std::strings with the name of
         *         each available serial port. 
         */
        std::vector<std::string> GetAvailableSerialPorts() const ;
        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout). If msTimeout
         *        is zero, then the method will block until all requested bytes
         *        are received. If numberOfBytes is zero and msTimeout is
         *        non-zero,  the method will continue receiving data for the
         *        specified of milliseconds. If numberOfBytes is zero and
         *        msTimeout is zero, the method will return immediately. In all
         *        cases, any data received remains available in the dataBuffer
         *        on return from this method.
         * @param dataBuffer The data buffer to place data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(DataBuffer&  dataBuffer,
                  size_t       numberOfBytes = 0,
                  size_t       msTimeout = 0) ;

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the specified
         *        number of milliseconds (msTimeout). If msTimeout is 0, then
         *        this method will block until all requested bytes are
         *        received. If numberOfBytes is zero, then this method will keep
         *        reading data till no more data is available at the serial port.
         *        In all cases, all read data is available in dataString on
         *        return from this method.
         * @param dataString The data string read from the serial port.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(std::string& dataString,
                  size_t       numberOfBytes = 0,
                  size_t       msTimeout = 0) ;

        /**
         * @brief Reads a single byte from the serial port.
         *        If no data is available within the specified number
         *        of milliseconds (msTimeout), then this method will
         *        throw a ReadTimeout exception. If msTimeout is 0,
         *        then this method will block until data is available.
         * @param charBuffer The character read from the serial port.
         * @param msTimeout The timeout period in milliseconds.
         */
        template <typename ByteType,
                  typename = std::enable_if_t<(sizeof(ByteType) == 1)>>
        void ReadByte(ByteType&  charBuffer, 
                      size_t msTimeout = 0) ;

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
        void ReadLine(std::string& dataString,
                      char         lineTerminator = '\n',
                      size_t       msTimeout = 0) ;

        /**
         * @brief Writes a DataBuffer to the serial port.
         * @param dataBuffer The DataBuffer to write to the serial port.
         */
        void Write(const DataBuffer& dataBuffer) ;

        /**
         * @brief Writes a std::string to the serial port.
         * @param dataString The std::string to be written to the serial port.
         */
        void Write(const std::string& dataString) ;

        /**
         * @brief Writes a single byte to the serial port.
         * @param charBuffer The byte to be written to the serial port.
         */
        void WriteByte(char charBuffer) ;

        /**
         * @brief Writes a single byte to the serial port.
         * @param charBuffer The byte to be written to the serial port.
         */
        void WriteByte(unsigned char charBuffer) ;

        /**
         * @brief Sets the current state of the serial port blocking status.
         * @param blockingStatus The serial port blocking status to be set,
         *        true if to be set blocking, false if to be set non-blocking.
         */
        void SetSerialPortBlockingStatus(bool blockingStatus) ;

        /**
         * @brief Gets the current state of the serial port blocking status.
         * @return True if port is blocking, false if port non-blocking.
         */
        bool GetSerialPortBlockingStatus() const ;

        /**
         * @brief Set the specified modem control line to the specified value.
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @param lineState State of the modem line after successful
         *        call to this method.
         */
        void SetModemControlLine(int modemLine,
                                 bool lineState) ;

        /**
         * @brief Get the current state of the specified modem control line.
         * @param modemLine One of the following four values: TIOCM_DTR,
         *        TIOCM_RTS, TIOCM_CTS, or TIOCM_DSR.
         * @return True if the specified line is currently set and false
         *         otherwise.
         */
        bool GetModemControlLine(int modemLine) ;

    private:

        /**
         * @brief Gets the bit rate for the serial port given the current baud rate setting.
         * @return Returns the bit rate the serial port is capable of achieving.
         */
        int GetBitRate(const BaudRate& baudRate) const ;

        /**
         * @brief Sets the default Linux specific line discipline modes.
         */
        void SetDefaultLinuxSpecificModes() ;

        /**
         * @brief Sets the default serial port input modes.
         */
        void SetDefaultInputModes() ;

        /**
         * @brief Sets the default serial port output modes.
         */
        void SetDefaultOutputModes() ;

        /**
         * @brief Sets the default serial port control modes.
         */
        void SetDefaultControlModes() ;

        /**
         * @brief Sets the default serial port local modes.
         */
        void SetDefaultLocalModes() ;

        /**
         * The file descriptor corresponding to the serial port.
         */
        int mFileDescriptor = -1 ;

        /**
         * The time in microseconds required for a byte of data to arrive at
         * the serial port.
         */
        int mByteArrivalTimeDelta = 1 ;

        /**
         * Serial port settings are saved into this struct immediately after
         * the port is opened. These settings are restored when the serial port
         * is closed.
         */
        termios mOldPortSettings {} ;
    } ;

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

    SerialPort::~SerialPort() noexcept
    {
        // Close the serial port if it is open.
        if (mImpl->IsOpen())
        {
            mImpl->Close() ;
        }
    }

    void
    SerialPort::Open(const std::string& fileName,
                     const std::ios_base::openmode& openMode)
    {
        mImpl->Open(fileName,
                    openMode) ;
    }

    void
    SerialPort::Close()
    {
        mImpl->Close() ;
    }

    void
    SerialPort::DrainWriteBuffer()
    {
        mImpl->DrainWriteBuffer() ;
    }

    void
    SerialPort::FlushInputBuffer()
    {
        mImpl->FlushInputBuffer() ;
    }

    void
    SerialPort::FlushOutputBuffer()
    {
        mImpl->FlushOutputBuffer() ;
    }

    void
    SerialPort::FlushIOBuffers()
    {
        mImpl->FlushIOBuffers() ;
    }

    bool
    SerialPort::IsDataAvailable() 
    {
        return mImpl->IsDataAvailable() ;
    }

    bool
    SerialPort::IsOpen() const
    {
        return mImpl->IsOpen() ;
    }

    void
    SerialPort::SetDefaultSerialPortParameters()
    {
        mImpl->SetDefaultSerialPortParameters() ;
    }

    void
    SerialPort::SetBaudRate(const BaudRate& baudRate)
    {
        mImpl->SetBaudRate(baudRate) ;
    }

    BaudRate
    SerialPort::GetBaudRate() const
    {
        return mImpl->GetBaudRate() ;
    }

    void
    SerialPort::SetCharacterSize(const CharacterSize& characterSize)
    {
        mImpl->SetCharacterSize(characterSize) ;
    }

    CharacterSize
    SerialPort::GetCharacterSize() const
    {
        return mImpl->GetCharacterSize() ;
    }

    void
    SerialPort::SetFlowControl(const FlowControl& flowControlType)
    {
        mImpl->SetFlowControl(flowControlType) ;
    }

    FlowControl
    SerialPort::GetFlowControl() const
    {
        return mImpl->GetFlowControl() ;
    }

    void
    SerialPort::SetParity(const Parity& parityType)
    {
        mImpl->SetParity(parityType) ;
    }

    Parity
    SerialPort::GetParity() const
    {
        return mImpl->GetParity() ;
    }

    void
    SerialPort::SetStopBits(const StopBits& stopBits)
    {
        mImpl->SetStopBits(stopBits) ;
    }

    StopBits
    SerialPort::GetStopBits() const
    {
        return mImpl->GetStopBits() ;
    }

    void
    SerialPort::SetVMin(const short vmin)
    {
        mImpl->SetVMin(vmin) ;
    }

    short
    SerialPort::GetVMin() const
    {
        return mImpl->GetVMin() ;
    }

    void
    SerialPort::SetVTime(const short vtime)
    {
        mImpl->SetVTime(vtime) ;
    }

    short
    SerialPort::GetVTime() const
    {
        return mImpl->GetVTime() ;
    }

    void
    SerialPort::SetDTR(const bool dtrState)
    {
        mImpl->SetDTR(dtrState) ;
    }

    bool
    SerialPort::GetDTR() const
    {
        return mImpl->GetDTR() ;
    }

    void
    SerialPort::SetRTS(const bool rtsState)
    {
        mImpl->SetRTS(rtsState) ;
    }

    bool
    SerialPort::GetRTS() const
    {
        return mImpl->GetRTS() ;
    }

    bool
    SerialPort::GetCTS() 
    {
        return mImpl->GetCTS() ;
    }

    bool
    SerialPort::GetDSR()
    {
        return mImpl->GetDSR() ;
    }

    int
    SerialPort::GetFileDescriptor() const
    {
        return mImpl->GetFileDescriptor() ;
    }

    int
    SerialPort::GetNumberOfBytesAvailable() 
    {
        return mImpl->GetNumberOfBytesAvailable() ;
    }

    std::vector<std::string>
    SerialPort::GetAvailableSerialPorts() const
    {
        return mImpl->GetAvailableSerialPorts() ;
    }

    void
    SerialPort::Read(DataBuffer& dataBuffer,
                     const size_t numberOfBytes,
                     const size_t msTimeout)
    {
        mImpl->Read(dataBuffer,
                    numberOfBytes,
                    msTimeout) ;
    }

    void
    SerialPort::Read(std::string& dataString,
                     const size_t numberOfBytes,
                     const size_t msTimeout)
    {
        mImpl->Read(dataString,
                    numberOfBytes,
                    msTimeout) ;
    }

    void
    SerialPort::ReadByte(char&        charBuffer,
                         const size_t msTimeout)
    {
        mImpl->ReadByte(charBuffer,
                        msTimeout) ;
    }

    void
    SerialPort::ReadByte(unsigned char& charBuffer,
                         const size_t   msTimeout)
    {
        mImpl->ReadByte(charBuffer,
                        msTimeout) ;
    }

    void
    SerialPort::ReadLine(std::string& dataString,
                         const char   lineTerminator,
                         const size_t msTimeout)
    {
        mImpl->ReadLine(dataString,
                        lineTerminator,
                        msTimeout) ;
    }

    void
    SerialPort::Write(const DataBuffer& dataBuffer)
    {
        mImpl->Write(dataBuffer) ;
    }

    void
    SerialPort::Write(const std::string& dataString)
    {
        mImpl->Write(dataString) ;
    }

    void
    SerialPort::WriteByte(const char charBuffer)
    {
        mImpl->WriteByte(charBuffer) ;
    }

    void
    SerialPort::WriteByte(const unsigned char charBuffer)
    {
        mImpl->WriteByte(charBuffer) ;
    }

    void 
    SerialPort::SetSerialPortBlockingStatus(const bool blockingStatus)
    {
        mImpl->SetSerialPortBlockingStatus(blockingStatus) ;
    }

    bool 
    SerialPort::GetSerialPortBlockingStatus() const
    {
        return mImpl->GetSerialPortBlockingStatus() ;
    }

    void 
    SerialPort::SetModemControlLine(const int modemLine,
                                    const bool lineState)
    {
        mImpl->SetModemControlLine(modemLine, lineState) ;
    }

    bool 
    SerialPort::GetModemControlLine(const int modemLine) 
    {
        return mImpl->GetModemControlLine(modemLine) ;
    }

    /** -------------------------- Implementation -------------------------- */

    inline
    SerialPort::Implementation::Implementation(const std::string&   fileName,
                                               const BaudRate&      baudRate,
                                               const CharacterSize& characterSize,
                                               const FlowControl&   flowControlType,
                                               const Parity&        parityType,
                                               const StopBits&      stopBits)
    {
        this->Open(fileName, std::ios_base::in | std::ios_base::out) ;
        this->SetBaudRate(baudRate) ;
        this->SetCharacterSize(characterSize) ;
        this->SetFlowControl(flowControlType) ;
        this->SetParity(parityType) ;
        this->SetStopBits(stopBits) ;
    }

    inline
    SerialPort::Implementation::~Implementation()
    {
        // Close the serial port if it is open.
        if (this->IsOpen())
        {
            this->Close() ;
        }
    }

    inline
    void
    SerialPort::Implementation::Open(const std::string& fileName,
                                     const std::ios_base::openmode& openMode)
    {
        // Throw an exception if the port is already open.
        if (this->IsOpen())
        {
            throw AlreadyOpen(ERR_MSG_PORT_ALREADY_OPEN) ;
        }

        // We only allow three different combinations of ios_base::openmode so
        // we can use a switch here to construct the flags to be used with the
        // open() system call.  Since we are dealing with the serial port we
        // need to use the O_NOCTTY option.
        int flags = (O_NOCTTY | O_NONBLOCK) ;   // NOLINT (hicpp-signed-bitwise)

        if (openMode == (std::ios_base::in | std::ios_base::out))
        {
            flags |= O_RDWR ;
        } 
        else if (openMode == std::ios_base::in)
        {
            flags |= O_RDONLY ;
        } 
        else if (openMode == std::ios_base::out)
        {
            flags |= O_WRONLY ;
        }
        else
        {
            throw OpenFailed {"Invalid or unsupported open mode"} ;
        }

        // Try to open the serial port. 
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        mFileDescriptor = call_with_retry(open, fileName.c_str(), flags) ;
        
        if (this->mFileDescriptor < 0)
        {
            close(this->mFileDescriptor) ;
            throw OpenFailed(std::strerror(errno)) ;
        }

        // Set the serial port to exclusive access to this process.
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        if (call_with_retry(ioctl,
                            this->mFileDescriptor,
                            TIOCEXCL) == -1)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Save the current settings of the serial port so they can be
        // restored when the serial port is closed.
        if (tcgetattr(this->mFileDescriptor,
                      &mOldPortSettings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }

        // Set up the default configuration for the serial port.
        this->SetDefaultSerialPortParameters() ;

        // Flush the input and output buffers associated with the port.
        this->FlushIOBuffers() ;
    }

    inline
    void
    SerialPort::Implementation::Close()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Restore the old settings of the port.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &mOldPortSettings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Otherwise, close the serial port and set the file descriptor
        // to an invalid value.
        if (close(this->mFileDescriptor) < 0) 
        {
            throw std::runtime_error(std::strerror(errno)) ;
        } 

        // Set the file descriptor to an invalid value, -1. 
        mFileDescriptor = -1 ;
    }

    inline
    void
    SerialPort::Implementation::DrainWriteBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (tcdrain(this->mFileDescriptor) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::FlushInputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (tcflush(this->mFileDescriptor, TCIFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::FlushOutputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (tcflush(this->mFileDescriptor, TCOFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::FlushIOBuffers()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (tcflush(this->mFileDescriptor, TCIOFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    bool
    SerialPort::Implementation::IsOpen() const
    {
        return (this->mFileDescriptor != -1) ;
    }

    inline
    bool
    SerialPort::Implementation::IsDataAvailable() 
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        int number_of_bytes_available = 0 ;
        bool is_data_available = false ;

        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        const auto ioctl_result = call_with_retry(ioctl,
                                                  this->mFileDescriptor,
                                                  FIONREAD,
                                                  &number_of_bytes_available) ;
        
        if ((ioctl_result >= 0) and
            (number_of_bytes_available > 0))
        {
            is_data_available = true ;
        }

        return is_data_available ;
    }

    inline
    void 
    SerialPort::Implementation::SetDefaultSerialPortParameters()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        #ifdef __linux__
            SetDefaultLinuxSpecificModes() ;
        #endif

        SetDefaultInputModes() ;
        SetDefaultOutputModes() ;
        SetDefaultControlModes() ;
        SetDefaultLocalModes() ;

        SetBaudRate(BaudRate::BAUD_DEFAULT) ;
        SetCharacterSize(CharacterSize::CHAR_SIZE_DEFAULT) ;
        SetFlowControl(FlowControl::FLOW_CONTROL_DEFAULT) ;
        SetParity(Parity::PARITY_DEFAULT) ;
        SetStopBits(StopBits::STOP_BITS_DEFAULT) ;
        SetVMin(VMIN_DEFAULT) ;
        SetVTime(VTIME_DEFAULT) ;
    }

    inline
    void
    SerialPort::Implementation::SetBaudRate(const BaudRate& baudRate)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Set the baud rate for both input and output.
        if (0 != cfsetspeed(&port_settings, static_cast<speed_t>(baudRate)))
        {
            // If applying the baud rate settings fail, throw an exception.
            throw std::runtime_error(ERR_MSG_INVALID_BAUD_RATE) ;
        }

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            // If applying the settings fails, throw an exception.
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Set the time interval value (us) required for one byte of data to arrive.
        mByteArrivalTimeDelta = (BITS_PER_BYTE * MICROSECONDS_PER_SEC) / GetBitRate(baudRate) ;
    }

    inline
    BaudRate
    SerialPort::Implementation::GetBaudRate() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Read the input and output baud rates.
        const auto input_baud = cfgetispeed(&port_settings) ;
        const auto output_baud = cfgetospeed(&port_settings) ;

        // Make sure that the input and output baud rates are
        // equal. Otherwise, we do not know which one to return.
        if (input_baud != output_baud)
        {
            throw std::runtime_error(ERR_MSG_INVALID_BAUD_RATE) ;
            // return BaudRate::BAUD_INVALID ;
        }

        // Obtain the input baud rate from the current settings.
        return BaudRate(input_baud) ;
    }

    inline
    int
    SerialPort::Implementation::GetBitRate(const BaudRate& baudRate) const
    {
        int baud_rate_as_int = 1 ;

        switch (baudRate)
        {
        case BaudRate::BAUD_50:
            baud_rate_as_int = 50 ;
            break ;

        case BaudRate::BAUD_75:
            baud_rate_as_int = 75 ;
            break ;

        case BaudRate::BAUD_110:
            baud_rate_as_int = 110 ;
            break ;

        case BaudRate::BAUD_134:
            baud_rate_as_int = 134 ;
            break ;

        case BaudRate::BAUD_150:
            baud_rate_as_int = 150 ;
            break ;

        case BaudRate::BAUD_200:
            baud_rate_as_int = 200 ;
            break ;

        case BaudRate::BAUD_300:
            baud_rate_as_int = 300 ;
            break ;

        case BaudRate::BAUD_600:
            baud_rate_as_int = 600 ;
            break ;

        case BaudRate::BAUD_1200:
            baud_rate_as_int = 1200 ;
            break ;

        case BaudRate::BAUD_1800:
            baud_rate_as_int = 1800 ;
            break ;

        case BaudRate::BAUD_2400:
            baud_rate_as_int = 2400 ;
            break ;

        case BaudRate::BAUD_4800:
            baud_rate_as_int = 4800 ;
            break ;

        case BaudRate::BAUD_9600:
            baud_rate_as_int = 9600 ;
            break ;

        case BaudRate::BAUD_19200:
            baud_rate_as_int = 19200 ;
            break ;

        case BaudRate::BAUD_38400:
            baud_rate_as_int = 38400 ;
            break ;

        case BaudRate::BAUD_57600:
            baud_rate_as_int = 57600 ;
            break ;

        case BaudRate::BAUD_115200:
            baud_rate_as_int = 115200 ;
            break ;

        case BaudRate::BAUD_230400:
            baud_rate_as_int = 230400 ;
            break ;

        case BaudRate::BAUD_460800:
            baud_rate_as_int = 460800 ;
            break ;

        case BaudRate::BAUD_500000:
            baud_rate_as_int = 500000 ;
            break ;

        case BaudRate::BAUD_576000:
            baud_rate_as_int = 576000 ;
            break ;

        case BaudRate::BAUD_921600:
            baud_rate_as_int = 921600 ;
            break ;

        case BaudRate::BAUD_1000000:
            baud_rate_as_int = 1000000 ;
            break ;

        case BaudRate::BAUD_1152000:
            baud_rate_as_int = 1152000 ;
            break ;

        case BaudRate::BAUD_1500000:
            baud_rate_as_int = 1500000 ;
            break ;

        case BaudRate::BAUD_2000000:
            baud_rate_as_int = 2000000 ;
            break ;

        case BaudRate::BAUD_2500000:
            baud_rate_as_int = 2500000 ;
            break ;

        case BaudRate::BAUD_3000000:
            baud_rate_as_int = 3000000 ;
            break ;

        case BaudRate::BAUD_3500000:
            baud_rate_as_int = 3500000 ;
            break ;

        case BaudRate::BAUD_4000000:
            baud_rate_as_int = 4000000 ;
            break ;
        default:
            // If an incorrect baud rate was specified, throw an exception.
            throw std::runtime_error(ERR_MSG_INVALID_BAUD_RATE) ;
        }

        return baud_rate_as_int ;
    }

    inline
    void
    SerialPort::Implementation::SetCharacterSize(const CharacterSize& characterSize)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(ERR_MSG_INVALID_CHARACTER_SIZE) ;
        }

        // Set the character size to the specified value. If the character
        // size is not 8 then it is also important to set ISTRIP. Setting
        // ISTRIP causes all but the 7 low-order bits to be set to
        // zero. Otherwise they are set to unspecified values and may
        // cause problems. At the same time, we should clear the ISTRIP
        // flag when the character size is 8 otherwise the MSB will always
        // be set to zero (ISTRIP does not check the character size
        // setting;it just sets every bit above the low 7 bits to zero).
        if (characterSize == CharacterSize::CHAR_SIZE_8)
        {
            // NOLINTNEXTLINE (hicpp-signed-bitwise)
            port_settings.c_iflag &= ~ISTRIP ;  // Clear the ISTRIP flag.
        }
        else
        {
            port_settings.c_iflag |= ISTRIP ;   // Set the ISTRIP flag.
        }

        // Set the character size.
        // NOLINTNEXTLINE (hicpp-signed-bitwise)
        port_settings.c_cflag &= ~CSIZE ;                               // Clear all CSIZE bits.
        port_settings.c_cflag |= static_cast<tcflag_t>(characterSize) ; // Set the character size.

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    CharacterSize
    SerialPort::Implementation::GetCharacterSize() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Read the character size from the setttings.
        // NOLINTNEXTLINE (hicpp-signed-bitwise)
        return CharacterSize(port_settings.c_cflag & CSIZE) ;
    }

    inline
    void
    SerialPort::Implementation::SetFlowControl(const FlowControl& flowControlType)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Flush the input and output buffers associated with the port.
        if (tcflush(this->mFileDescriptor,
                    TCIOFLUSH) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(ERR_MSG_INVALID_FLOW_CONTROL) ;
        }

        // Set the flow control. Hardware flow control uses the RTS (Ready
        // To Send) and CTS (clear to Send) lines. Software flow control
        // uses IXON|IXOFF
        switch(flowControlType)
        {
        case FlowControl::FLOW_CONTROL_HARDWARE:
            port_settings.c_iflag &= ~ (IXON|IXOFF) ;   // NOLINT (hicpp-signed-bitwise)
            port_settings.c_cflag |= CRTSCTS ;
            port_settings.c_cc[VSTART] = _POSIX_VDISABLE ;
            port_settings.c_cc[VSTOP] = _POSIX_VDISABLE ;
            break ;
        case FlowControl::FLOW_CONTROL_SOFTWARE:
            port_settings.c_iflag |= IXON|IXOFF ;        // NOLINT(hicpp-signed-bitwise)
            port_settings.c_cflag &= ~CRTSCTS ;
            port_settings.c_cc[VSTART] = CTRL_Q ;        // 0x11 (021) ^q
            port_settings.c_cc[VSTOP]  = CTRL_S ;        // 0x13 (023) ^s
            break ;
        case FlowControl::FLOW_CONTROL_NONE:
            port_settings.c_iflag &= ~(IXON|IXOFF) ;    // NOLINT(hicpp-signed-bitwise)
            port_settings.c_cflag &= ~CRTSCTS ;
            break ;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_FLOW_CONTROL) ;
            // break ; break not needed after a throw
        }
        
        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    FlowControl
    SerialPort::Implementation::GetFlowControl() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Check if IXON and IXOFF are set in c_iflag. If both are set and
        // VSTART and VSTOP are set to 0x11 (^Q) and 0x13 (^S) respectively,
        // then we are using software flow control.
        if ((port_settings.c_iflag & IXON) and // NOLINT (hicpp-signed-bitwise)
            (port_settings.c_iflag & IXOFF) and // NOLINT (hicpp-signed-bitwise)
            (CTRL_Q == port_settings.c_cc[VSTART]) and
            (CTRL_S == port_settings.c_cc[VSTOP]))
        {
            return FlowControl::FLOW_CONTROL_SOFTWARE ;
        }

        if (not ((port_settings.c_iflag & IXON) or // NOLINT (hicpp-signed-bitwise)
                 (port_settings.c_iflag & IXOFF))) // NOLINT (hicpp-signed-bitwise)
        {
            if (0 != (port_settings.c_cflag & CRTSCTS))
            {
                // If neither IXON or IXOFF is set then we must have hardware flow
                // control.
                return FlowControl::FLOW_CONTROL_HARDWARE ;
            }
            return FlowControl::FLOW_CONTROL_NONE ;
        }

        // If none of the above conditions are satisfied then the serial port
        // is using a flow control setup which we do not support at present.
        return FlowControl::FLOW_CONTROL_INVALID ;
    }

    inline
    void
    SerialPort::Implementation::SetParity(const Parity& parityType)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Set the parity type
        switch(parityType)
        {
        case Parity::PARITY_EVEN:
            port_settings.c_cflag |= PARENB ;
            port_settings.c_cflag &= ~PARODD ;  // NOLINT (hicpp-signed-bitwise)
            port_settings.c_iflag |= INPCK ;
            break ;
        case Parity::PARITY_ODD:
            port_settings.c_cflag |= PARENB ;
            port_settings.c_cflag |= PARODD ;
            port_settings.c_iflag |= INPCK ;
            break ;
        case Parity::PARITY_NONE:
            port_settings.c_cflag &= ~PARENB ;  // NOLINT (hicpp-signed-bitwise)
            port_settings.c_iflag |= IGNPAR ;
            break ;
        default:
            throw std::invalid_argument(ERR_MSG_INVALID_PARITY) ;
            // break ; break not needed after a throw
        }

        // Apply the modified port settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    Parity
    SerialPort::Implementation::GetParity() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Get the parity setting from the termios structure. 
        if (0 != (port_settings.c_cflag & PARENB)) // NOLINT (hicpp-signed-bitwise)
        {
            // parity is enabled.
            if (port_settings.c_cflag & PARODD) // NOLINT (hicpp-signed-bitwise)
            {
                return Parity::PARITY_ODD ; // odd parity
            }
            return Parity::PARITY_EVEN ;    // even parity
        }
        return Parity::PARITY_NONE ;        // no parity.
    }

    inline
    void
    SerialPort::Implementation::SetStopBits(const StopBits& stopBits)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Set the number of stop bits.
        switch(stopBits)
        {
        case StopBits::STOP_BITS_1:
            port_settings.c_cflag &= ~CSTOPB ;  // NOLINT (hicpp-signed-bitwise)
            break ;
        case StopBits::STOP_BITS_2:
            port_settings.c_cflag |= CSTOPB ;
            break ;
        default: 
            throw std::invalid_argument(ERR_MSG_INVALID_STOP_BITS) ;
            // break ; break not needed after a throw
        }

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    StopBits
    SerialPort::Implementation::GetStopBits() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // If CSTOPB is set then we are using two stop bits, otherwise we
        // are using 1 stop bit.
        if (port_settings.c_cflag & CSTOPB) // NOLINT (hicpp-signed-bitwise)
        {
            return StopBits::STOP_BITS_2 ;
        }
        return StopBits::STOP_BITS_1 ;
    }

    inline
    void 
    SerialPort::Implementation::SetVMin(const short vmin)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (vmin < 0 || vmin > 255)
        {
            throw std::invalid_argument(std::strerror(errno)) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        port_settings.c_cc[VMIN] = static_cast<cc_t>(vmin) ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    short 
    SerialPort::Implementation::GetVMin() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        return port_settings.c_cc[VMIN] ;
    }

    inline
    void 
    SerialPort::Implementation::SetVTime(const short vtime)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if (vtime < 0 || vtime > 255)
        {
            throw std::invalid_argument(std::strerror(errno)) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        port_settings.c_cc[VTIME] = static_cast<cc_t>(vtime) ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    short 
    SerialPort::Implementation::GetVTime() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        return port_settings.c_cc[VTIME] ;
    }

    inline
    void
    SerialPort::Implementation::SetDTR(const bool dtrState)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        this->SetModemControlLine(TIOCM_DTR, 
                                   dtrState) ;
    }

    inline
    bool
    SerialPort::Implementation::GetDTR() 
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        return this->GetModemControlLine(TIOCM_DTR) ;
    }    

    inline
    void
    SerialPort::Implementation::SetRTS(const bool rtsState)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        this->SetModemControlLine(TIOCM_RTS, 
                                  rtsState) ;
    }

    inline
    bool
    SerialPort::Implementation::GetRTS()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        return this->GetModemControlLine(TIOCM_RTS) ;
    }    

    inline
    bool
    SerialPort::Implementation::GetCTS()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        return this->GetModemControlLine(TIOCM_CTS) ;
    }    

    inline
    bool
    SerialPort::Implementation::GetDSR() 
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        return this->GetModemControlLine(TIOCM_DSR) ;
    }

    inline
    int
    SerialPort::Implementation::GetFileDescriptor() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        return this->mFileDescriptor ;
    }

    inline
    int
    SerialPort::Implementation::GetNumberOfBytesAvailable()
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        int number_of_bytes_available = 0 ;

        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        if (call_with_retry(ioctl,
                            this->mFileDescriptor,
                            FIONREAD,
                            &number_of_bytes_available) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        return number_of_bytes_available ;
    }

    inline
    std::vector<std::string>
    SerialPort::Implementation::GetAvailableSerialPorts() const
    {
        constexpr int array_size = 3 ;
        constexpr size_t max_port_number = 128 ;
        
        std::string serial_ports[array_size] = {"/dev/ttyS",
                                                "/dev/ttyACM",
                                                "/dev/ttyUSB"} ;

        std::vector<std::string> serial_port_names {} ;
        for (const auto& port_prefix: serial_ports)
        {
            for (size_t j = 0 ;j < max_port_number;j++)
            {
                const auto file_name = port_prefix + std::to_string(j) ;

                // Try to open the serial port. 
                const auto file_desc = call_with_retry(open,
                                                       file_name.c_str(), 
                                                       O_RDWR | O_NOCTTY | O_NONBLOCK) ; // NOLINT (hicpp-vararg)

                if (file_desc > 0)
                {
                    serial_struct serial_port_info {} ;
                    // NOLINTNEXTLINE (hicpp-vararg)
                    if (call_with_retry(ioctl,
                                        file_desc,
                                        TIOCGSERIAL,
                                        &serial_port_info) == -1)
                    {
                        throw std::runtime_error(std::strerror(errno)) ;
                    }

                    serial_port_names.push_back(file_name) ;

                    close(file_desc) ;
                }
            }
        }

        return serial_port_names ;
    }

    inline
    void
    SerialPort::Implementation::SetModemControlLine(const int  modemLine,
                                                    const bool lineState)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
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
            throw std::invalid_argument(std::strerror(errno)) ;
        }

        // Set or unset the specified bit according to the value of lineState.
        int ioctl_result = -1 ;
        
        if (lineState)
        {
            int set_line_mask = modemLine ;
            // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
            ioctl_result = call_with_retry(ioctl,
                                           this->mFileDescriptor, 
                                           TIOCMBIS,
                                           &set_line_mask) ;
        }
        else
        {
            int reset_line_mask = modemLine ;
            // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
            ioctl_result = call_with_retry(ioctl,
                                           this->mFileDescriptor, 
                                           TIOCMBIC,
                                           &reset_line_mask) ;
        }

        // Check for errors.
        if (ioctl_result < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }
    }

    inline
    bool
    SerialPort::Implementation::GetModemControlLine(const int modemLine) 
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
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
            throw std::invalid_argument(std::strerror(errno)) ;
        }
        
        // Use an ioctl() call to get the state of the line.
        int serial_port_state = 0 ;
        
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        if (call_with_retry(ioctl,
                            this->mFileDescriptor,
                            TIOCMGET,
                            &serial_port_state) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        return (0 != (serial_port_state & modemLine)) ; // NOLINT(hicpp-signed-bitwise)
    }

    inline
    void
    SerialPort::Implementation::SetSerialPortBlockingStatus(const bool blockingStatus)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        int flags = fcntl(this->mFileDescriptor, F_GETFL, 0) ;
        
        if (blockingStatus)
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg, hicpp-vararg)
            if (fcntl(this->mFileDescriptor, 
                      F_SETFL, 
                      flags &~ O_NONBLOCK) < 0) // NOLINT (hicpp-signed-bitwise)
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
        else
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg, hicpp-vararg)
            if (fcntl(this->mFileDescriptor, 
                      F_SETFL, 
                      flags | O_NONBLOCK) < 0) // NOLINT (hicpp-signed-bitwise)
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
    }

    inline
    bool
    SerialPort::Implementation::GetSerialPortBlockingStatus() const
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        bool blocking_status = false ;

        int flags = fcntl(this->mFileDescriptor, F_GETFL, 0) ;
        
        if (flags == -1)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        if (flags == (flags | O_NONBLOCK)) // NOLINT (hicpp-signed-bitwise)
        {
            blocking_status = true ;
        }

        return blocking_status ;
    }

    inline
    void
    SerialPort::Implementation::SetDefaultLinuxSpecificModes()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // @NOTE - termios.c_line is not a standard element of the termios
        // structure, (as per the Single Unix Specification 3).
        port_settings.c_line = '\0' ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::SetDefaultInputModes()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Ignore Break conditions on input.
        port_settings.c_iflag = IGNBRK ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::SetDefaultOutputModes()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        port_settings.c_oflag = 0 ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::SetDefaultControlModes()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        // Enable the receiver (CREAD) and ignore modem control lines (CLOCAL).
        port_settings.c_cflag |= CREAD | CLOCAL ;    // NOLINT (hicpp-signed-bitwise)

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::SetDefaultLocalModes()
    {
        // Make sure that the serial port is open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Get the current serial port settings.
        termios port_settings {} ;
        std::memset(&port_settings, 0, sizeof(port_settings)) ;
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno)) ;
        }

        port_settings.c_lflag = 0 ;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno)) ;
        }
    }

    inline
    void
    SerialPort::Implementation::Read(DataBuffer&  dataBuffer,
                                     const size_t numberOfBytes,
                                     const size_t msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if ((numberOfBytes == 0) and
            (msTimeout == 0))
        {
            return ;
        }

        // Local variables.
        size_t number_of_bytes_read = 0 ;
        size_t number_of_bytes_remaining = std::max(numberOfBytes, 1UL) ;
        size_t maximum_number_of_bytes = dataBuffer.max_size() ;
        
        // Clear the data buffer and reserve enough space in the buffer to store the incoming data.
        dataBuffer.clear() ;
        dataBuffer.resize(number_of_bytes_remaining) ;

        // Obtain the entry time.
        const auto entry_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

        while (number_of_bytes_remaining > 0)
        {
            // If insufficient space remains in the buffer, exit the loop and return .
            if (number_of_bytes_remaining >= maximum_number_of_bytes - number_of_bytes_read)
            {
                break ;
            }

            if (numberOfBytes == 0)
            {
                // Add an additional element for the read() call.
                dataBuffer.resize(number_of_bytes_read + 1) ;
            }

            const auto read_result = call_with_retry(read,
                                                     this->mFileDescriptor,
                                                     &dataBuffer[number_of_bytes_read],
                                                     number_of_bytes_remaining) ;

            if (read_result > 0)
            {
                number_of_bytes_read += read_result ;

                if (numberOfBytes != 0)
                {
                    number_of_bytes_remaining = numberOfBytes - number_of_bytes_read ;

                    if (number_of_bytes_remaining == 0)
                    {
                        break ;
                    }
                }
            }
            else if ((read_result <= 0) and
                     (errno != EWOULDBLOCK))
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }

            // Obtain the current time.
            const auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

            // Calculate the time delta.
            const auto elapsed_time = current_time - entry_time ;

            // Calculate the elapsed number of milliseconds.
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() ;

            // If more than msTimeout milliseconds have elapsed while
            // waiting for data, then we throw a ReadTimeout exception.
            if ((msTimeout > 0) and
                (static_cast<size_t>(elapsed_ms) > msTimeout))
            {
                // Resize the data buffer.
                dataBuffer.resize(number_of_bytes_read) ;

                throw ReadTimeout(ERR_MSG_READ_TIMEOUT) ;
            }
            
            // Allow sufficient time for an additional byte to arrive.
            usleep(mByteArrivalTimeDelta) ;
        }
    }

    inline
    void
    SerialPort::Implementation::Read(std::string& dataString,
                                     const size_t numberOfBytes,
                                     const size_t msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        if ((numberOfBytes == 0) and
            (msTimeout == 0))
        {
            return ;
        }

        // Local variables.
        size_t number_of_bytes_read = 0 ;
        size_t number_of_bytes_remaining = std::max(numberOfBytes, 1UL) ;
        size_t maximum_number_of_bytes = dataString.max_size() ;
        
        // Clear the data buffer and reserve enough space in the buffer to store the incoming data.
        dataString.clear() ;
        dataString.resize(number_of_bytes_remaining) ;

        // Obtain the entry time.
        const auto entry_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

        while (number_of_bytes_remaining > 0)
        { 
            // If insufficient space remains in the buffer, exit the loop and return .
            if (number_of_bytes_remaining >= maximum_number_of_bytes - number_of_bytes_read)
            {
                break ;
            }

            if (numberOfBytes == 0)
            {
                // Add an additional element for the read() call.
                dataString.resize(number_of_bytes_read + 1) ;
            }

            const auto read_result = call_with_retry(read,
                                                     this->mFileDescriptor,
                                                     &dataString[number_of_bytes_read],
                                                     number_of_bytes_remaining) ;

            if (read_result > 0)
            {
                number_of_bytes_read += read_result ;

                if (numberOfBytes != 0)
                {
                    number_of_bytes_remaining = numberOfBytes - number_of_bytes_read ;

                    if (number_of_bytes_remaining == 0)
                    {
                        break ;
                    }
                }
            }
            else if (read_result <= 0 &&
                     errno != EWOULDBLOCK)
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }

            // Obtain the current time.
            const auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

            // Calculate the time delta.
            const auto elapsed_time = current_time - entry_time ;

            // Calculate the elapsed number of milliseconds.
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() ;

            // If more than msTimeout milliseconds have elapsed while
            // waiting for data, then we throw a ReadTimeout exception.
            if ((msTimeout > 0) and
                (static_cast<size_t>(elapsed_ms) > msTimeout))
            {
                // Resize the data string.
                dataString.resize(number_of_bytes_read) ;

                throw ReadTimeout(ERR_MSG_READ_TIMEOUT) ;
            }

            // Allow sufficient time for an additional byte to arrive.
            usleep(mByteArrivalTimeDelta) ;
        }
    }

    template <typename ByteType, typename /* unused */>
    inline
    void 
    SerialPort::Implementation::ReadByte(ByteType&  charBuffer, 
                                         const size_t msTimeout) 
    {
        // Double check to make sure that ByteType is exactly one byte long.
        static_assert(sizeof(ByteType) == 1, 
                      "ByteType must have a size of exactly one byte.") ;

        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Obtain the entry time.
        const auto entry_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

        // Loop until the number of bytes requested have been read or the
        // timeout has elapsed.
        ssize_t read_result = 0 ;
        while (read_result < 1)
        {
            read_result = call_with_retry(read,
                                          this->mFileDescriptor,
                                          &charBuffer,
                                          sizeof(ByteType)) ;
            
            // If the byte has been successfully read, exit the loop and return.
            if (read_result == sizeof(ByteType))
            {
                break ;
            }

            if ((read_result <= 0) and
                (errno != EWOULDBLOCK))
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }

            // Obtain the current time.
            const auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

            // Calculate the time delta.
            const auto elapsed_time = current_time - entry_time ;

            // Calculate the elapsed number of milliseconds.
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() ;

            // Throw a ReadTimeout exception if more than msTimeout milliseconds
            // have elapsed while waiting for data.
            if ((msTimeout > 0) and
                (static_cast<size_t>(elapsed_ms) > msTimeout))
            {
                throw ReadTimeout(ERR_MSG_READ_TIMEOUT) ;
            }

            // Allow sufficient time for an additional byte to arrive.
            usleep(mByteArrivalTimeDelta) ;
        }
    }

    inline
    void
    SerialPort::Implementation::ReadLine(std::string& dataString,
                                         const char   lineTerminator,
                                         const size_t msTimeout)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Clear the data string.
        dataString.clear() ;

        unsigned char next_char = 0 ;

        size_t elapsed_ms = 0 ;
        
        ssize_t remaining_ms = 0 ;

        std::chrono::high_resolution_clock::duration entry_time ;
        std::chrono::high_resolution_clock::duration current_time ;
        std::chrono::high_resolution_clock::duration elapsed_time ;

        // Obtain the entry time.
        entry_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

        while (next_char != lineTerminator)
        {
            // Obtain the current time.
            current_time = std::chrono::high_resolution_clock::now().time_since_epoch() ;

            // Calculate the time delta.
            elapsed_time = current_time - entry_time ;

            // Calculate the elapsed number of milliseconds.
            elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() ;

            // If more than msTimeout milliseconds have elapsed while
            // waiting for data, then we throw a ReadTimeout exception.
            if (msTimeout > 0 &&
                elapsed_ms > msTimeout)
            {
                throw ReadTimeout(ERR_MSG_READ_TIMEOUT) ;
            }

            remaining_ms = msTimeout - elapsed_ms ;

            this->ReadByte(next_char,
                           remaining_ms) ;

            dataString += next_char ;
        }
    }

    inline
    void
    SerialPort::Implementation::Write(const DataBuffer& dataBuffer)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        size_t number_of_bytes = dataBuffer.size() ;

        // Nothing needs to be done if there is no data in the string.
        if (number_of_bytes <= 0)
        {
            return ;
        }

        // Local variables.
        size_t number_of_bytes_written = 0 ;
        size_t number_of_bytes_remaining = number_of_bytes ;

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        ssize_t write_result = 0 ;
        
        while (number_of_bytes_remaining > 0)
        {
            write_result = call_with_retry(write,
                                           this->mFileDescriptor,
                                           &dataBuffer[number_of_bytes_written],
                                           number_of_bytes_remaining) ;

            if (write_result >= 0)
            {
                number_of_bytes_written += write_result ;
                number_of_bytes_remaining = number_of_bytes - number_of_bytes_written ;

                if (number_of_bytes_remaining == 0)
                {
                    break ;
                }
            }
            else if (write_result <= 0 &&
                     errno != EWOULDBLOCK)
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
    }

    inline
    void
    SerialPort::Implementation::Write(const std::string& dataString)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        size_t number_of_bytes = dataString.size() ;

        // Nothing needs to be done if there is no data in the string.
        if (number_of_bytes <= 0)
        {
            return ;
        }

        // Local variables.
        size_t number_of_bytes_written = 0 ;
        size_t number_of_bytes_remaining = number_of_bytes ;

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        ssize_t write_result = 0 ;
        
        while (number_of_bytes_remaining > 0)
        {
            write_result = call_with_retry(write,
                                           this->mFileDescriptor,
                                           &dataString[number_of_bytes_written],
                                           number_of_bytes_remaining) ;

            if (write_result >= 0)
            {
                number_of_bytes_written += write_result ;
                number_of_bytes_remaining = number_of_bytes - number_of_bytes_written ;

                if (number_of_bytes_remaining == 0)
                {
                    break ;
                }
            }
            else if (write_result <= 0 &&
                     errno != EWOULDBLOCK)
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
    }

    inline
    void
    SerialPort::Implementation::WriteByte(const char charBuffer)
    {
        // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        ssize_t write_result = 0 ;
        
        while (write_result <= 0)
        {
            write_result = call_with_retry(write,
                                           this->mFileDescriptor,
                                           &charBuffer,
                                           1) ;

            if (write_result == 1)
            {
                break ;
            }

            if ((write_result <= 0) and
                (errno != EWOULDBLOCK))
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
    }

    inline
    void
    SerialPort::Implementation::WriteByte(const unsigned char charBuffer)
    {
                // Throw an exception if the serial port is not open.
        if (not this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Write the data to the serial port. Keep retrying if EAGAIN
        // error is received and EWOULDBLOCK is not received.
        ssize_t write_result = 0 ;
        
        while (write_result <= 0)
        {
            write_result = call_with_retry(write,
                                           this->mFileDescriptor,
                                           &charBuffer,
                                           1) ;

            if (write_result == 1)
            {
                break ;
            }

            if ((write_result <= 0) and
                (errno != EWOULDBLOCK))
            {
                throw std::runtime_error(std::strerror(errno)) ;
            }
        }
    }
} // namespace LibSerial
