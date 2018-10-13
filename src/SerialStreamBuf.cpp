/******************************************************************************
 * @file SerialStreamBuf.cpp                                                  *
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

#include "SerialStreamBuf.h"
#include "SerialPort.h"

#include <cassert>
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
        Implementation() = default ;

        /**
         * @brief Default Destructor.
         */
        ~Implementation() = default ;

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
                       const StopBits&      stopBits) ;

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
        bool GetDTR() const ;

        /**
         * @brief Sets the serial port RTS line status.
         * @param rtsState The state to set the RTS line
         */
        void SetRTS(bool rtsState) ;

        /**
         * @brief Gets the serial port RTS line status.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS() const ;

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
         * @brief Writes up to n characters from the character sequence at
         *        char s to the serial port associated with the buffer.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters that were successfully
         *         written to the serial port.
         */
        std::streamsize xsputn(const char_type* character,
                               std::streamsize numberOfBytes) ;

        /**
         * @brief Reads up to n characters from the serial port and returns
         *        them through the character array located at s.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters actually read from the
         *         serial port.
         */
        std::streamsize xsgetn(char_type* character,
                               std::streamsize numberOfBytes) ;

        /**
         * @brief Writes the specified character to the associated serial port.
         * @param character The character to be written to the serial port.
         * @return Returns the character.
         */
        std::streambuf::int_type overflow(int_type character) ;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for buffered I/O while uflow() is called
         *        for unbuffered I/O.
         * @return Returns the next character from the serial port.
         */
        std::streambuf::int_type underflow() ;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for unbuffered I/O while underflow() is
         *        called for unbuffered I/O.
         * @return Returns the next character from the serial port.
         */
        std::streambuf::int_type uflow() ;

        /**
         * @brief This function is called when a putback of a character
         *        fails. This must be implemented for unbuffered I/O as all
         *        streambuf subclasses are required to provide putback of
         *        at least one character.
         * @param character The character to putback.
         * @return Returns The character iff successful, otherwise eof to signal an error.
         */
        std::streambuf::int_type pbackfail(int_type character) ;

        /**
         * @brief Checks whether input is available on the port.
         * @return Returns 1 if characters are available at the serial port,
         *         0 if no characters are available, and -1 if unsuccessful.
         */
        std::streamsize  showmanyc() ;

        /**
         * @brief True if a putback value is available in mPutbackChar.
         *
         * :TODO: Should this be private?
         */
        bool mPutbackAvailable {false} ;

        /**
         * @brief We use unbuffered I/O for the serial port. However, we
         *        need to provide the putback of at least one character.
         *        This character contains the putback character.
         *
         * :TODO: Should this be private?
         */
        char mPutbackChar {0} ;

    private:

        /**
         * SerialPort device that will be used for communication.
         */
        SerialPort mSerialPort {} ;
    } ;

    SerialStreamBuf::SerialStreamBuf()
        : mImpl(new Implementation)
    {
        setbuf(nullptr, 0) ;
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
        setbuf(nullptr, 0) ;
    }

    SerialStreamBuf::~SerialStreamBuf()
    {
        // Close the serial port if it is open.
        if (mImpl->IsOpen())
        {
            mImpl->Close() ;
        }
    }

    void
    SerialStreamBuf::Open(const std::string& fileName,
                          const std::ios_base::openmode& openMode)
    {
        mImpl->Open(fileName,
                    openMode) ;
    }

    void
    SerialStreamBuf::Close()
    {
        mImpl->Close() ;
    }

    void
    SerialStreamBuf::DrainWriteBuffer()
    {
        mImpl->DrainWriteBuffer() ;
    }

    void
    SerialStreamBuf::FlushInputBuffer()
    {
        mImpl->FlushInputBuffer() ;
    }

    void
    SerialStreamBuf::FlushOutputBuffer()
    {
        mImpl->FlushOutputBuffer() ;
    }

    void
    SerialStreamBuf::FlushIOBuffers()
    {
        mImpl->FlushIOBuffers() ;
    }

    bool
    SerialStreamBuf::IsDataAvailable()
    {
        return mImpl->IsDataAvailable() ;
    }

    bool
    SerialStreamBuf::IsOpen() const
    {
        return mImpl->IsOpen() ;
    }

    void
    SerialStreamBuf::SetDefaultSerialPortParameters()
    {
        mImpl->SetDefaultSerialPortParameters() ;
    }

    void
    SerialStreamBuf::SetBaudRate(const BaudRate& baudRate)
    {
        mImpl->SetBaudRate(baudRate) ;
    }

    BaudRate
    SerialStreamBuf::GetBaudRate() const
    {
        return mImpl->GetBaudRate() ;
    }

    void
    SerialStreamBuf::SetCharacterSize(const CharacterSize& characterSize)
    {
        mImpl->SetCharacterSize(characterSize) ;
    }

    CharacterSize
    SerialStreamBuf::GetCharacterSize() const
    {
        return mImpl->GetCharacterSize() ;
    }

    void
    SerialStreamBuf::SetFlowControl(const FlowControl& flowControlType)
    {
        mImpl->SetFlowControl(flowControlType) ;
    }

    FlowControl
    SerialStreamBuf::GetFlowControl() const
    {
        return mImpl->GetFlowControl() ;
    }

    void
    SerialStreamBuf::SetParity(const Parity& parityType)
    {
        mImpl->SetParity(parityType) ;
    }

    Parity
    SerialStreamBuf::GetParity() const
    {
        return mImpl->GetParity() ;
    }

    void
    SerialStreamBuf::SetStopBits(const StopBits& stopBits)
    {
        mImpl->SetStopBits(stopBits) ;
    }

    StopBits
    SerialStreamBuf::GetStopBits() const
    {
        return mImpl->GetStopBits() ;
    }

    void
    SerialStreamBuf::SetVMin(const short vmin)
    {
        mImpl->SetVMin(vmin) ;
    }

    short
    SerialStreamBuf::GetVMin() const
    {
        return mImpl->GetVMin() ;
    }

    void
    SerialStreamBuf::SetVTime(const short vtime)
    {
        mImpl->SetVTime(vtime) ;
    }

    short
    SerialStreamBuf::GetVTime() const
    {
        return mImpl->GetVTime() ;
    }

    void
    SerialStreamBuf::SetDTR(const bool dtrState)
    {
        mImpl->SetDTR(dtrState) ;
    }

    bool
    SerialStreamBuf::GetDTR() const
    {
        return mImpl->GetDTR() ;
    }

    void
    SerialStreamBuf::SetRTS(const bool rtsState)
    {
        mImpl->SetRTS(rtsState) ;
    }

    bool
    SerialStreamBuf::GetRTS() const
    {
        return mImpl->GetRTS() ;
    }

    bool
    SerialStreamBuf::GetCTS()
    {
        return mImpl->GetCTS() ;
    }

    bool
    SerialStreamBuf::GetDSR()
    {
        return mImpl->GetDSR() ;
    }

    int
    SerialStreamBuf::GetFileDescriptor() const
    {
        return mImpl->GetFileDescriptor() ;
    }

    int
    SerialStreamBuf::GetNumberOfBytesAvailable()
    {
        return mImpl->GetNumberOfBytesAvailable() ;
    }

    std::vector<std::string>
    SerialStreamBuf::GetAvailableSerialPorts() const
    {
        return mImpl->GetAvailableSerialPorts() ;
    }

    std::streambuf*
    SerialStreamBuf::setbuf(char_type* character, std::streamsize numberOfBytes)
    {
        return std::streambuf::setbuf(character, numberOfBytes) ;
    }

    std::streamsize
    SerialStreamBuf::xsputn(const char_type* character, std::streamsize numberOfBytes)
    {
        return mImpl->xsputn(character, numberOfBytes) ;
    }

    std::streamsize
    SerialStreamBuf::xsgetn(char_type* character, std::streamsize numberOfBytes)
    {
        return mImpl->xsgetn(character, numberOfBytes) ;
    }

    std::streambuf::int_type
    SerialStreamBuf::overflow(const int_type character)
    {
        return mImpl->overflow(character) ;
    }

    std::streambuf::int_type
    SerialStreamBuf::underflow()
    {
        return mImpl->underflow() ;
    }

    std::streambuf::int_type
    SerialStreamBuf::uflow()
    {
        return mImpl->uflow() ;
    }

    std::streambuf::int_type
    SerialStreamBuf::pbackfail(const int_type character)
    {
        return mImpl->pbackfail(character) ;
    }

    std::streamsize
    SerialStreamBuf::showmanyc()
    {
        return mImpl->showmanyc() ;
    }


    /** -------------------------- Implementation -------------------------- */

    inline
    SerialStreamBuf::Implementation::Implementation(const std::string&   fileName,
                                                    const BaudRate&      baudRate,
                                                    const CharacterSize& characterSize,
                                                    const FlowControl&   flowControlType,
                                                    const Parity&        parityType,
                                                    const StopBits&      stopBits)
    try : mSerialPort(fileName,
                      baudRate, 
                      characterSize,
                      flowControlType,
                      parityType,
                      stopBits)
    {
        //  empty
    }
    catch (const std::exception& err)
    {
        throw OpenFailed(err.what()) ;
    }

    inline
    void
    SerialStreamBuf::Implementation::Open(const std::string& fileName,
                                          const std::ios_base::openmode& openMode)
    try
    {
        mSerialPort.Open(fileName, 
                         openMode) ;

        // @note - Stream communications need to happen in blocking mode.
        mSerialPort.SetSerialPortBlockingStatus(true) ;
    }
    catch (const AlreadyOpen&)
    {
        throw ;
    }
    catch (const OpenFailed&)
    {
        throw ;
    }
    catch (const std::exception& err)
    {
        throw OpenFailed(err.what()) ;
    }

    inline
    void
    SerialStreamBuf::Implementation::Close()
    {
        mSerialPort.Close() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::DrainWriteBuffer()
    {
        mSerialPort.DrainWriteBuffer() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushInputBuffer()
    {
        mSerialPort.FlushInputBuffer() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushOutputBuffer()
    {
        mSerialPort.FlushOutputBuffer() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::FlushIOBuffers()
    {
        mSerialPort.FlushIOBuffers() ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::IsOpen() const
    {
        return mSerialPort.IsOpen() ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::IsDataAvailable() 
    {
        return mSerialPort.IsDataAvailable() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDefaultSerialPortParameters()
    {
        mSerialPort.SetDefaultSerialPortParameters() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetBaudRate(const BaudRate& baudRate)
    {
        mSerialPort.SetBaudRate(baudRate) ;
    }

    inline
    BaudRate
    SerialStreamBuf::Implementation::GetBaudRate() const
    {
        return mSerialPort.GetBaudRate() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetCharacterSize(const CharacterSize& characterSize)
    {
        mSerialPort.SetCharacterSize(characterSize) ;
    }

    inline
    CharacterSize
    SerialStreamBuf::Implementation::GetCharacterSize() const
    {
        return mSerialPort.GetCharacterSize() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetFlowControl(const FlowControl& flowControlType)
    {
        mSerialPort.SetFlowControl(flowControlType) ;
    }

    inline
    FlowControl
    SerialStreamBuf::Implementation::GetFlowControl() const
    {
        return mSerialPort.GetFlowControl() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetParity(const Parity& parityType)
    {
        mSerialPort.SetParity(parityType) ;
    }

    inline
    Parity
    SerialStreamBuf::Implementation::GetParity() const
    {
        return mSerialPort.GetParity() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetStopBits(const StopBits& stopBits)
    {
        mSerialPort.SetStopBits(stopBits) ;
    }

    inline
    StopBits
    SerialStreamBuf::Implementation::GetStopBits() const
    {
        return mSerialPort.GetStopBits() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetVMin(const short vmin)
    {
        mSerialPort.SetVMin(vmin) ;
    }

    inline
    short
    SerialStreamBuf::Implementation::GetVMin() const
    {
        return mSerialPort.GetVMin() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetVTime(const short vtime)
    {
        mSerialPort.SetVTime(vtime) ;
    }

    inline
    short
    SerialStreamBuf::Implementation::GetVTime() const
    {
        return mSerialPort.GetVTime() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetDTR(const bool dtrState)
    {
        mSerialPort.SetDTR(dtrState) ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetDTR() const
    {
        return mSerialPort.GetDTR() ;
    }

    inline
    void
    SerialStreamBuf::Implementation::SetRTS(const bool rtsState)
    {
        mSerialPort.SetRTS(rtsState) ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetRTS() const
    {
        return mSerialPort.GetRTS() ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetCTS() 
    {
        return mSerialPort.GetCTS() ;
    }

    inline
    bool
    SerialStreamBuf::Implementation::GetDSR()
    {
        return mSerialPort.GetDSR() ;
    }

    inline
    int
    SerialStreamBuf::Implementation::GetFileDescriptor() const
    {
        return mSerialPort.GetFileDescriptor() ;
    }

    inline
    int
    SerialStreamBuf::Implementation::GetNumberOfBytesAvailable()
    {
        return mSerialPort.GetNumberOfBytesAvailable() ;
    }

    inline
    std::vector<std::string>
    SerialStreamBuf::Implementation::GetAvailableSerialPorts() const
    {
        return mSerialPort.GetAvailableSerialPorts() ;
    }

    inline
    std::streamsize
    SerialStreamBuf::Implementation::xsputn(const char_type* character,
                                            std::streamsize numberOfBytes)
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // If n is less than 1 there is nothing to accomplish.
        if (numberOfBytes <= 0)
        {
            return 0 ;
        }

        // Write the n characters to the serial port.
        // 
        // :TODO: Consider using mSerialPort.Write() here instead of writing
        //        to the file descriptor directly.
        const auto fd = mSerialPort.GetFileDescriptor() ;
        ssize_t result = call_with_retry(write, fd, character, numberOfBytes) ;

        // If the write failed then return 0.
        if (result <= 0)
        {
            return 0 ;
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
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // If n is less than 1 there is nothing to accomplish.
        if (numberOfBytes <= 0)
        {
            return 0 ;
        }

        if (character == nullptr) {
            return 0 ;
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
                // 
                // :TODO: Consider using mSerialPort.Read() here instead of
                // using the file descriptor.
                //
                const auto fd = mSerialPort.GetFileDescriptor() ;
                result = call_with_retry(read, fd, &character[1], numberOfBytes-1) ;

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
            // 
            // :TODO: Consider using mSerialPort.Read() here instead of
            //        using the file descriptor.
            const auto fd = mSerialPort.GetFileDescriptor() ;
            result = call_with_retry(read, fd, character, numberOfBytes) ;
        }

        // If result == -1 then the read call had an error, otherwise, if
        // result == 0 then we could not read the characters. In either
        // case, we return 0 to indicate that no characters could be read
        // from the serial port.
        if (result <= 0)
        {
            return 0 ;
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
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Try to write the specified character to the serial port.
        if (traits_type::eq_int_type(character, traits_type::eof()))
        {
            // If character is the eof character then we do nothing.
            return traits_type::eof() ;
        }

        // Otherwise we write the character to the serial port.
        // 
        // :TODO: Consider using a method of SerialPort class here instead
        //        of using the file descriptor.
        const auto fd = mSerialPort.GetFileDescriptor() ;
        char out_char = traits_type::to_char_type(character) ;
        ssize_t result = call_with_retry(write, fd, &out_char, 1) ;

        // If the write failed then return eof.
        if (result <= 0)
        {
            return traits_type::eof() ;
        }

        // Otherwise, return something other than eof().
        return traits_type::not_eof(character) ;
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::underflow()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // Read the next character from the serial port.
        char next_char = 0 ;
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
            // 
            // :TODO: Consider using a method of SerialPort class here instead
            // of using the file descriptor.
            //
            const auto fd = mSerialPort.GetFileDescriptor() ;
            result = call_with_retry(read, fd, &next_char, 1) ;

            // Make the next character the putback character. This has the
            // effect of returning the next character without changing gptr()
            // as required by the C++ standard.
            if (result == 1)
            {
                mPutbackChar = next_char;
                mPutbackAvailable = true ;
            }
            else if (result <= 0)
            {
                // If we had a problem reading the character, we return
                // traits::eof().
                return traits_type::eof() ;
            }
        }

        // :NOTE: Wed Aug  9 21:26:51 2000 Pagey
        //        The value of mPutbackAvailable is always true when the code
        //        reaches here.

        // Return the character as an int value as required by the C++
        // standard.
        return traits_type::to_int_type(next_char) ;
    }

    inline
    std::streambuf::int_type
    SerialStreamBuf::Implementation::uflow()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        int_type next_char = underflow() ;

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
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        // If a putback character is already available, then we cannot
        // do any more putback and hence need to return eof.
        if (mPutbackAvailable)
        {
            return traits_type::eof() ;
        }
        if (traits_type::eq_int_type(character, traits_type::eof()))
        {
            // If an eof character is passed in, then we are required to
            // backup one character. However, we cannot do this for a serial
            // port. Hence we return eof to signal an error.
            return traits_type::eof() ;
        }
        //
        // If no putback character is available at present, then make
        // "character" the putback character and return it.
        mPutbackChar = traits_type::to_char_type(character) ;
        mPutbackAvailable = true ;
        return traits_type::not_eof(character) ;
    }

    inline
    std::streamsize
    SerialStreamBuf::Implementation::showmanyc()
    {
        // Throw an exception if the serial port is not open.
        if (!this->IsOpen())
        {
            throw NotOpen(ERR_MSG_PORT_NOT_OPEN) ;
        }

        ssize_t number_of_bytes_available = 0 ;

        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        // 
        // :TODO: Consider using a method of SerialPort class here instead
        //        of using the file descriptor.
        const auto fd = mSerialPort.GetFileDescriptor() ;
        const auto result = call_with_retry(ioctl,
                                            fd,
                                            FIONREAD,
                                            &number_of_bytes_available) ;

        if ((result >= 0) and (number_of_bytes_available > 0))
        {
            mPutbackAvailable = true ;
        }

        return number_of_bytes_available;
    }
} // namespace LibSerial
