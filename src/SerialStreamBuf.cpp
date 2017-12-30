/******************************************************************************
 *   @file SerialStreamBuf.cc                                                 *
 *   @copyright (C) 2004 Manish Pagey                                         *
 *   crayzeewulf@users.sourceforge.net                                        *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU Lessser General Public License as          *
 *   published by the Free Software Foundation; either version 2 of the       *
 *   License, or (at your option) any later version.                          *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU Lesser General Public License for more details.                      *
 *                                                                            *
 *   You should have received a copy of the GNU Lesser General Public         *
 *   License along with this program; if not, write to the                    *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#include "SerialStreamBuf.h"

#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace LibSerial
{
    /**
     * @brief SerialStreamBuf::Implementation is the SerialStreamBuf implementation class.
     */
    class SerialStreamBuf::Implementation
    {
    public:
        /**
         * @brief Default Constructor.
         */
        Implementation();

        /**
         * @brief Constructor that allows a SerialStreamBuf instance to be 
         *        created and opened, initializing the corresponding
         *        serial port with the specified parameters.
         * @param fileName The file name of the serial stream.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial stream.
         * @param stopBits The number of stop bits for the serial stream.
         * @param flowControlType The flow control type for the serial stream.
         */
        Implementation(const std::string&   fileName,
                       const BaudRate&      baudRate,
                       const CharacterSize& characterSize,
                       const FlowControl&   flowControlType,
                       const Parity&        parityType,
                       const StopBits&      stopBits);

        /**
         * @brief Default Destructor for a SerialStreamBuf object. Closes the
         *        serial port associated with mFileDescriptor if open.
         */
        ~Implementation();

        /**
         * @brief Opens the serial port associated with the specified
         *        file name and the specified mode.
         * @param fileName The file name of the serial port.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName,
                  const std::ios_base::openmode& openMode);

        /**
         * @brief Closes the serial port. All settings of the serial port will be
         *        lost and no more I/O can be performed on the serial port.
         */
        void Close();

        /**
         * @brief Waits until the write buffer is drained and then returns.
         */
        void DrainWriteBuffer();

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
        void SetFlowControl(const FlowControl& flowControlType);

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
         * @param stopBits The number of stop bits to set.
         */
        void SetStopBits(const StopBits& stopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetStopBits();

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
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDTR();

        /**
         * @brief Sets the serial port RTS line status.
         * @param rtsState The state to set the RTS line
         */
        void SetRTS(const bool rtsState);

        /**
         * @brief Gets the serial port RTS line status.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS();

        /**
         * @brief Gets the serial port CTS line status.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCTS();

        /**
         * @brief Gets the serial port DSR line status.
         * @return Returns true iff the status of the DSR line is high.
         */
        bool GetDSR();

        /**
         * @brief Gets the serial port file descriptor.
         * @return Returns the serial port file descriptor.
         */
        int GetFileDescriptor();

        /**
         * @brief Gets the number of bytes available in the read buffer.
         * @return Returns the number of bytes avilable in the read buffer.
         */
        int GetNumberOfBytesAvailable();

        /**
         * @brief Gets a list of available serial ports.
         * @return Returns a std::vector of std::strings with the name of
         *         each available serial port. 
         */
        std::vector<std::string> GetAvailableSerialPorts();

        /**
         * @brief Writes up to n characters from the character sequence at 
         *        char s to the serial port associated with the buffer.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters that were successfully
         *         written to the serial port. 
         */
        std::streamsize xsputn(const char_type* character,
                               std::streamsize numberOfBytes);

        /**
         * @brief Reads up to n characters from the serial port and returns
         *        them through the character array located at s.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters actually read from the
         *         serial port. 
         */
        std::streamsize xsgetn(char_type* character,
                               std::streamsize numberOfBytes);

        /**
         * @brief Writes the specified character to the associated serial port.
         * @param character The character to be written to the serial port.
         * @return Returns the character. 
         */
        std::streambuf::int_type overflow(const int_type character);

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for buffered I/O while uflow() is called
         *        for unbuffered I/O.
         * @return Returns the next character from the serial port.
         */
        std::streambuf::int_type underflow();

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for unbuffered I/O while underflow() is
         *        called for unbuffered I/O.
         * @return Returns the next character from the serial port.
         */
        std::streambuf::int_type uflow();

        /**
         * @brief This function is called when a putback of a character
         *        fails. This must be implemented for unbuffered I/O as all
         *        streambuf subclasses are required to provide putback of
         *        at least one character.
         * @param character The character to putback.
         * @return Returns The character iff successful, otherwise eof to signal an error.
         */
        std::streambuf::int_type pbackfail(const int_type character);

        /**
         * @brief Checks whether input is available on the port.
         * @return Returns 1 if characters are available at the serial port,
         *         0 if no characters are available, and -1 if unsuccessful.
         */
        std::streamsize  showmanyc();

        /** 
         * @brief True if a putback value is available in mPutbackChar. 
         */
        bool mPutbackAvailable;

        /** 
         * @brief We use unbuffered I/O for the serial port. However, we
         *        need to provide the putback of at least one character.
         *        This character contains the putback character.
         */
        char mPutbackChar;

    protected:

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
        void SetSerialPortBlockingStatus(const bool blockingStatus);

        /**
         * @brief Gets the current state of the serial port blocking status.
         * @return True if port is blocking, false if port non-blocking.
         */
        bool GetSerialPortBlockingStatus();

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

    SerialStreamBuf::SerialStreamBuf()
        : mImpl(new Implementation)
    {
        setbuf(0, 0);
        return;
    }

    SerialStreamBuf::SerialStreamBuf(const std::string&   fileName,
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
        setbuf(0, 0);
        return;
    }

    SerialStreamBuf::~SerialStreamBuf()
    {
        // Close the serial port if it is open.
        if (mImpl->IsOpen())
        {
            mImpl->Close();
        }

        return;
    }

    void
    SerialStreamBuf::Open(const std::string& fileName,
                          const std::ios_base::openmode& openMode)
    {
        mImpl->Open(fileName,
                    openMode);
        return;
    }

    void
    SerialStreamBuf::Close()
    {
        mImpl->Close();
        return;
    }

    void
    SerialStreamBuf::DrainWriteBuffer()
    {
        mImpl->DrainWriteBuffer();
        return;
    }

    void
    SerialStreamBuf::FlushInputBuffer()
    {
        mImpl->FlushInputBuffer();
        return;
    }

    void
    SerialStreamBuf::FlushOutputBuffer()
    {
        mImpl->FlushOutputBuffer();
        return;
    }

    void
    SerialStreamBuf::FlushIOBuffers()
    {
        mImpl->FlushIOBuffers();
        return;
    }

    bool
    SerialStreamBuf::IsDataAvailable()
    {
        return mImpl->IsDataAvailable();
    }

    bool
    SerialStreamBuf::IsOpen()
    {
        return mImpl->IsOpen();
    }

    void
    SerialStreamBuf::SetDefaultSerialPortParameters()
    {
        mImpl->SetDefaultSerialPortParameters();
        return;
    }

    void
    SerialStreamBuf::SetBaudRate(const BaudRate& baudRate)
    {
        mImpl->SetBaudRate(baudRate);
        return;
    }

    BaudRate
    SerialStreamBuf::GetBaudRate()
    {
        return mImpl->GetBaudRate();
    }

    void
    SerialStreamBuf::SetCharacterSize(const CharacterSize& characterSize)
    {
        mImpl->SetCharacterSize(characterSize);
        return;
    }

    CharacterSize
    SerialStreamBuf::GetCharacterSize()
    {
        return mImpl->GetCharacterSize();
    }

    void
    SerialStreamBuf::SetFlowControl(const FlowControl& flowControlType)
    {
        mImpl->SetFlowControl(flowControlType);
        return;
    }

    FlowControl
    SerialStreamBuf::GetFlowControl()
    {
        return mImpl->GetFlowControl();
    }

    void
    SerialStreamBuf::SetParity(const Parity& parityType)
    {
        mImpl->SetParity(parityType);
        return;
    }

    Parity
    SerialStreamBuf::GetParity()
    {
        return mImpl->GetParity();
    }

    void
    SerialStreamBuf::SetStopBits(const StopBits& stopBits)
    {
        mImpl->SetStopBits(stopBits);
        return;
    }

    StopBits
    SerialStreamBuf::GetStopBits()
    {
        return mImpl->GetStopBits();
    }

    void
    SerialStreamBuf::SetVMin(const short vmin)
    {
        mImpl->SetVMin(vmin);
        return;
    }

    short
    SerialStreamBuf::GetVMin()
    {
        return mImpl->GetVMin();
    }

    void
    SerialStreamBuf::SetVTime(const short vtime)
    {
        mImpl->SetVTime(vtime);
        return;
    }

    short
    SerialStreamBuf::GetVTime()
    {
        return mImpl->GetVTime();
    }

    void
    SerialStreamBuf::SetDTR(const bool dtrState)
    {
        mImpl->SetDTR(dtrState);
        return;
    }

    bool
    SerialStreamBuf::GetDTR()
    {
        return mImpl->GetDTR();
    }

    void
    SerialStreamBuf::SetRTS(const bool rtsState)
    {
        mImpl->SetRTS(rtsState);
        return;
    }

    bool
    SerialStreamBuf::GetRTS()
    {
        return mImpl->GetRTS();
    }

    bool
    SerialStreamBuf::GetCTS()
    {
        return mImpl->GetCTS();
    }

    bool
    SerialStreamBuf::GetDSR()
    {
        return mImpl->GetDSR();
    }

    int
    SerialStreamBuf::GetFileDescriptor()
    {
        return mImpl->GetFileDescriptor();
    }

    int
    SerialStreamBuf::GetNumberOfBytesAvailable()
    {
        return mImpl->GetNumberOfBytesAvailable();
    }

    std::vector<std::string>
    SerialStreamBuf::GetAvailableSerialPorts()
    {
        return mImpl->GetAvailableSerialPorts();
    }

    std::streambuf*
    SerialStreamBuf::setbuf(char_type* character, std::streamsize numberOfBytes)
    {
        return std::streambuf::setbuf(character, numberOfBytes);
    }

    std::streamsize
    SerialStreamBuf::xsputn(const char_type* character, std::streamsize numberOfBytes)
    {
        return mImpl->xsputn(character, numberOfBytes);
    }

    std::streamsize
    SerialStreamBuf::xsgetn(char_type* character, std::streamsize numberOfBytes)
    {
        return mImpl->xsgetn(character, numberOfBytes);
    }

    std::streambuf::int_type
    SerialStreamBuf::overflow(const int_type character)
    {
        return mImpl->overflow(character);
    }

    std::streambuf::int_type
    SerialStreamBuf::underflow()
    {
        return mImpl->underflow();
    }

    std::streambuf::int_type
    SerialStreamBuf::uflow()
    {
        return mImpl->uflow();
    }

    std::streambuf::int_type
    SerialStreamBuf::pbackfail(const int_type character)
    {
        return mImpl->pbackfail(character);
    }

    std::streamsize
    SerialStreamBuf::showmanyc()
    {
        return mImpl->showmanyc();
    }


    /** ------------------------------------------------------------ */
    inline
    SerialStreamBuf::Implementation::Implementation()
        : mPutbackAvailable(false)
        , mPutbackChar(0)
        , mFileDescriptor(-1)
    {
        /* Empty */
    }

    inline
    SerialStreamBuf::Implementation::Implementation(const std::string&   fileName,
                                                    const BaudRate&      baudRate,
                                                    const CharacterSize& characterSize,
                                                    const FlowControl&   flowControlType,
                                                    const Parity&        parityType,
                                                    const StopBits&      stopBits)
        : mPutbackAvailable(false)
        , mPutbackChar(0)
        , mFileDescriptor(-1)
    {
        this->Open(fileName, std::ios_base::in | std::ios_base::out);
        this->SetBaudRate(baudRate);
        this->SetCharacterSize(characterSize);
        this->SetFlowControl(flowControlType);
        this->SetParity(parityType);
        this->SetStopBits(stopBits);
        return;
    }

    inline
    SerialStreamBuf::Implementation::~Implementation()
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
    SerialStreamBuf::Implementation::Open(const std::string& fileName,
                                          const std::ios_base::openmode& openMode)
    {
        // Throw an exception if the port is already open.
        if (this->IsOpen())
        {
            throw AlreadyOpen(ERR_MSG_PORT_ALREADY_OPEN);
        }

        // We only allow three different combinations of ios_base::openmode so we can
        // use a switch here to construct the flags to be used with the open() system call.
        // Since we are dealing with the serial port we need to use the O_NOCTTY option.
        int flags = (O_NOCTTY | O_NONBLOCK);

        if (openMode == (std::ios_base::in | std::ios_base::out))
        {
            flags |= O_RDWR;
        } 
        else if (openMode == std::ios_base::in)
        {
            flags |= O_RDONLY;
        } 
        else if (openMode == std::ios_base::out)
        {
            flags |= O_WRONLY;
        }
        else
        {
            return;
        }

        // Try to open the serial port. 
        mFileDescriptor = open(fileName.c_str(), flags);
        
        if (this->mFileDescriptor < 0)
        {
            close(this->mFileDescriptor);
            throw OpenFailed(std::strerror(errno));
        }

        // Set the serial port to exclusive access to this process.
        if (ioctl(this->mFileDescriptor,
                  TIOCEXCL) == -1)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Save the current settings of the serial port so they can be
        // restored when the serial port is closed.
        if (tcgetattr(this->mFileDescriptor,
                      &mOldPortSettings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        // Set up the default configuration for the serial port.
        this->SetDefaultSerialPortParameters();

        // Flush the input and output buffers associated with the port.
        this->FlushIOBuffers();

        // @note - Stream communications need to happen in blocking mode.
        this->SetSerialPortBlockingStatus(true);

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::Close()
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
            throw std::runtime_error(std::strerror(errno));
        }

        // Otherwise, close the serial port and set the file descriptor
        // to an invalid value.
        if (close(this->mFileDescriptor) < 0) 
        {
            throw std::runtime_error(std::strerror(errno));
        } 

        // Set the file descriptor to an invalid value, -1. 
        mFileDescriptor = -1;
        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::DrainWriteBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcdrain(this->mFileDescriptor) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushInputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCIFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushOutputBuffer()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCOFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushIOBuffers()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (tcflush(this->mFileDescriptor, TCIOFLUSH) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    bool
    SerialStreamBuf::Implementation::IsOpen()
    {
        return (this->mFileDescriptor != -1);
    }

    inline
    bool
    SerialStreamBuf::Implementation::IsDataAvailable()
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
    SerialStreamBuf::Implementation::SetDefaultSerialPortParameters()
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
        SetStopBits(StopBits::STOP_BITS_DEFAULT);
        SetVMin(VMIN_DEFAULT);
        SetVTime(VTIME_DEFAULT);

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetBaudRate(const BaudRate& baudRate)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Set the baud rate for both input and output.
        if (cfsetspeed(&port_settings, (speed_t)baudRate))
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
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    BaudRate
    SerialStreamBuf::Implementation::GetBaudRate()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
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
    SerialStreamBuf::Implementation::SetCharacterSize(const CharacterSize& characterSize)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
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
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    CharacterSize
    SerialStreamBuf::Implementation::GetCharacterSize()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Read the character size from the setttings.
        return CharacterSize(port_settings.c_cflag & CSIZE);
    }

    inline
    void
    SerialStreamBuf::Implementation::SetFlowControl(const FlowControl& flowControlType)
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
            throw OpenFailed(std::strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
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
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    FlowControl
    SerialStreamBuf::Implementation::GetFlowControl()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
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
    SerialStreamBuf::Implementation::SetParity(const Parity& parityType)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
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
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    Parity
    SerialStreamBuf::Implementation::GetParity()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
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
    SerialStreamBuf::Implementation::SetStopBits(const StopBits& stopBits)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Set the number of stop bits.
        switch(stopBits)
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
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    StopBits
    SerialStreamBuf::Implementation::GetStopBits()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
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
    SerialStreamBuf::Implementation::SetVMin(const short vmin)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (vmin < 0 || vmin > 255)
        {
            throw std::invalid_argument(std::strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        port_settings.c_cc[VMIN] = (cc_t)vmin;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    short
    SerialStreamBuf::Implementation::GetVMin()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return port_settings.c_cc[VMIN];
    }

    inline
    void
    SerialStreamBuf::Implementation::SetVTime(const short vtime)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        if (vtime < 0 || vtime > 255)
        {
            throw std::invalid_argument(std::strerror(errno));
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        port_settings.c_cc[VTIME] = (cc_t)vtime;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    short
    SerialStreamBuf::Implementation::GetVTime()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return port_settings.c_cc[VTIME];
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDTR(const bool dtrState)
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
    SerialStreamBuf::Implementation::GetDTR()
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
    SerialStreamBuf::Implementation::SetRTS(const bool rtsState)
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
    SerialStreamBuf::Implementation::GetRTS()
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
    SerialStreamBuf::Implementation::GetCTS()
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
    SerialStreamBuf::Implementation::GetDSR()
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
    SerialStreamBuf::Implementation::GetFileDescriptor()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        return this->mFileDescriptor;
    }

    inline
    int
    SerialStreamBuf::Implementation::GetNumberOfBytesAvailable()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        int number_of_bytes_available = 0;

        if (ioctl(this->mFileDescriptor,
                  FIONREAD,
                  &number_of_bytes_available) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return number_of_bytes_available;
    }

    inline
    std::vector<std::string>
    SerialStreamBuf::Implementation::GetAvailableSerialPorts()
    {
        const int array_size = 3;
        int file_descriptor = -1;
        size_t max_port_number = 128;
        
        serial_struct serial_port_info;

        std::string file_name;
        file_name.clear();

        std::vector<std::string> serial_port_names;
        serial_port_names.clear();

        std::string serial_ports[array_size] = {"/dev/ttyS",
                                                "/dev/ttyACM",
                                                "/dev/ttyUSB"};

        for (size_t i = 0; i < array_size; i++)
        {
            for (size_t j = 0; j < max_port_number; j++)
            {
                file_name = serial_ports[i] + std::to_string(j);

                // Try to open the serial port. 
                file_descriptor = open(file_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

                if (file_descriptor > 0)
                {
                    if (ioctl(file_descriptor,
                              TIOCGSERIAL,
                              &serial_port_info) == -1)
                    {
                        throw std::runtime_error(std::strerror(errno));
                    }

                    serial_port_names.push_back(file_name);

                    close(file_descriptor);
                }
            }
        }

        return serial_port_names;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetModemControlLine(const int  modemLine,
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
            throw std::invalid_argument(std::strerror(errno));
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
        if (ioctl_result < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return;
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetModemControlLine(const int modemLine)
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
            throw std::invalid_argument(std::strerror(errno));
        }
        
        // Use an ioctl() call to get the state of the line.
        int serial_port_state = 0;
        
        if (ioctl(this->mFileDescriptor,
                  TIOCMGET,
                  &serial_port_state) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        return (serial_port_state & modemLine);
    }

    inline
    void
    SerialStreamBuf::Implementation::SetSerialPortBlockingStatus(const bool blockingStatus)
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
                throw std::runtime_error(std::strerror(errno));
            }
        }
        else
        {
            if (fcntl(this->mFileDescriptor, 
                      F_SETFL, 
                      flags | O_NONBLOCK) < 0)
            {
                throw std::runtime_error(std::strerror(errno));
            }
        }
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetSerialPortBlockingStatus()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        bool blocking_status = false;

        int flags = fcntl(this->mFileDescriptor, F_GETFL, 0);
        
        if (flags == -1)
        {
            throw std::runtime_error(std::strerror(errno));
        }
        else if (flags == (flags | O_NONBLOCK))
        {
            blocking_status = true;
        }

        return blocking_status;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultLinuxSpecificModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // @NOTE - termios.c_line is not a standard element of the termios
        // structure, (as per the Single Unix Specification 3).
        port_settings.c_line = '\0';

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultInputModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Ignore Break conditions on input.
        port_settings.c_iflag = IGNBRK;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultOutputModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        port_settings.c_oflag = 0;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultControlModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        // Enable the receiver (CREAD) and ignore modem control lines (CLOCAL).
        port_settings.c_cflag |= CREAD | CLOCAL;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        return;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultLocalModes()
    {
        // Make sure that the serial port is open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Get the current serial port settings.
        termios port_settings;
        std::memset(&port_settings, 0, sizeof(port_settings));
        
        if (tcgetattr(this->mFileDescriptor,
                      &port_settings) < 0)
        {
            throw std::runtime_error(std::strerror(errno));
        }

        port_settings.c_lflag = 0;

        // Apply the modified settings.
        if (tcsetattr(this->mFileDescriptor,
                      TCSANOW,
                      &port_settings) < 0)
        {
            throw OpenFailed(std::strerror(errno));
        }

        return;
    }

    inline
    std::streamsize
    SerialStreamBuf::Implementation::xsputn(const char_type* character,
                                            std::streamsize numberOfBytes)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }
        
        // If n is less than 1 there is nothing to accomplish.
        if (numberOfBytes <= 0)
        {
            return 0;
        }

        // Write the n characters to the serial port. 
        ssize_t result = write(this->mFileDescriptor, character, numberOfBytes);

        // If the write failed then return 0. 
        if (result <= 0)
        {
            return 0;
        }

        // Otherwise, return the number of bytes actually written.
        return result;
    }

    inline
    std::streamsize
    SerialStreamBuf::Implementation::xsgetn(char_type* character,
                                            std::streamsize numberOfBytes)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // If n is less than 1 there is nothing to accomplish.
        if (numberOfBytes <= 0)
        {
            return 0;
        }

        // Try to read up to n characters in the array s.
        ssize_t result = -1;

        // If a putback character is available, then we need to read only
        // n-1 character.
        if (mPutbackAvailable)
        {
            // Put the mPutbackChar at the beginning of the array 's'.
            // Increment result to indicate that a character has been placed in s.
            character[0] = mPutbackChar;
            result++;

            // The putback character is no longer available. 
            mPutbackAvailable = false;

            // If we need to read more than one character, then call read()
            // and try to read numberOfBytes-1 more characters and put them
            // at location starting from &character[1].
            if (numberOfBytes > 1)
            {
                result = read(this->mFileDescriptor, &character[1], numberOfBytes-1);

                // If read was successful, then we need to increment result by
                // one to indicate that the putback character was prepended to
                // the array, s. If read failed then leave result at -1.
                if (result != -1)
                {
                    result++;
                }
            }
        }
        else
        {
            // If no putback character is available then we try to
            // read numberOfBytes characters.
            result = read(this->mFileDescriptor, character, numberOfBytes);
        }

        // If result == -1 then the read call had an error, otherwise, if
        // result == 0 then we could not read the characters. In either
        // case, we return 0 to indicate that no characters could be read
        // from the serial port.
        if (result <= 0)
        {
            return 0;
        }

        // Return the number of characters actually read from the serial port.
        return result;
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::overflow(const int_type character)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Try to write the specified character to the serial port. 
        if (traits_type::eq_int_type(character, traits_type::eof()))
        {
            // If character is the eof character then we do nothing. 
            return traits_type::eof();
        }
        else
        {
            // Otherwise we write the character to the serial port. 
            char out_char = traits_type::to_char_type(character);
            ssize_t result = write(this->mFileDescriptor, &out_char, 1);

            // If the write failed then return eof. 
            if (result <= 0)
            {
                return traits_type::eof();
            }

            // Otherwise, return something other than eof().
            return traits_type::not_eof(character);
        }
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::underflow()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // Read the next character from the serial port. 
        char next_char = 0;
        ssize_t result = -1;

        // If a putback character is available then we return that
        // character. However, we are not supposed to change the value of
        // gptr() in this routine so we leave mPutbackAvailable set to true.
        if (mPutbackAvailable)
        {
            next_char = mPutbackChar;
        }
        else
        {
            // If no putback character is available then we need to read one
            // character from the serial port.
            result = read(this->mFileDescriptor, &next_char, 1);

            // Make the next character the putback character. This has the
            // effect of returning the next character without changing gptr()
            // as required by the C++ standard.
            if (result == 1)
            {
                mPutbackChar = next_char;
                mPutbackAvailable = true;
            }
            else if (result <= 0)
            {
                // If we had a problem reading the character, we return
                // traits::eof().
                return traits_type::eof();
            }
        }

        // :NOTE: Wed Aug  9 21:26:51 2000 Pagey
        // The value of mPutbackAvailable is always true when the code
        // reaches here.

        // Return the character as an int value as required by the C++
        // standard.
        return traits_type::to_int_type(next_char);
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::uflow()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        int_type next_char = underflow();

        mPutbackAvailable = false;

        return next_char;
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::pbackfail(const int_type character)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        // If a putback character is already available, then we cannot
        // do any more putback and hence need to return eof.
        if (mPutbackAvailable)
        {
            return traits_type::eof();
        }
        else if (traits_type::eq_int_type(character, traits_type::eof()))
        {
            // If an eof character is passed in, then we are required to
            // backup one character. However, we cannot do this for a serial
            // port. Hence we return eof to signal an error.
            return traits_type::eof();
        }
        else
        {
            // If no putback character is available at present, then make
            // "character" the putback character and return it.
            mPutbackChar = traits_type::to_char_type(character);
            mPutbackAvailable = true;
            return traits_type::not_eof(character);
        }
    }

    inline
    std::streamsize
    SerialStreamBuf::Implementation::showmanyc()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN);
        }

        ssize_t result = -1;
        ssize_t number_of_bytes_available = 0;

        result = ioctl(this->mFileDescriptor,
                       FIONREAD,
                       &number_of_bytes_available);

        if (result >= 0 &&
            number_of_bytes_available > 0)
        {
            mPutbackAvailable = true;
        }

        return number_of_bytes_available;
    }
}