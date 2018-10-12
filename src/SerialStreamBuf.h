/******************************************************************************
 * @file SerialStreamBuf.h                                                    *
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

#pragma once

#include <SerialPortConstants.h>

#include <memory>
#include <streambuf>
#include <vector>

namespace LibSerial
{
    /**
     * @brief SerialStreamBuf is the streambuf subclass used by SerialStream.
     *        This subclass takes care of opening the serial port file in the
     *        required modes and providing the corresponding file descriptor
     *        to SerialStream so that various parameters associated with the
     *        serial port can be set. Several features of this streambuf class
     *        resemble those of std::filebuf, however this class it not made a
     *        subclass of filebuf because we need access to the file descriptor
     *        associated with the serial port and the standard filebuf does not
     *        provide access to it.
     *
     *        At present, this class uses unbuffered I/O and all calls
     *        to setbuf() will be ignored.
     */
    class SerialStreamBuf : public std::streambuf
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit SerialStreamBuf() ;

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
        explicit SerialStreamBuf(const std::string&   fileName,
                                 const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                                 const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                                 const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                                 const Parity&        parityType      = Parity::PARITY_DEFAULT,
                                 const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT) ;

        /**
         * @brief Default Destructor for a SerialStreamBuf object. Closes the
         *        serial port associated with mFileDescriptor if open.
         */
        virtual ~SerialStreamBuf() ;

        /**
         * @brief Copy construction is disallowed.
         */
        SerialStreamBuf(const SerialStreamBuf& otherSerialStreamBuf) = delete ;

        /**
         * @brief Move construction is disallowed.
         */
        SerialStreamBuf(const SerialStreamBuf&& otherSerialStreamBuf) = delete ;

        /**
         * @brief Copy assignment is disallowed.
         */
        SerialStreamBuf& operator=(const SerialStreamBuf& otherSerialStreamBuf) = delete ;

        /**
         * @brief Move assignment is disallowed.
         */
        SerialStreamBuf& operator=(const SerialStreamBuf&& otherSerialStreamBuf) = delete ;

        /**
         * @brief Opens the serial port associated with the specified
         *        file name and the specified mode.
         * @param fileName The file name of the serial port.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName,
                  const std::ios_base::openmode& openMode = std::ios_base::in | std::ios_base::out) ;

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
         * @brief Checks if data is available at the input of the serial port.
         * @return Returns true iff data is available to read.
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
         * @brief Gets the current flow control setting.
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
         * @note See VMIN in man termios(3).
         * @param vmin the number of minimum characters to be set.
         */
        void SetVMin(short vmin) ;

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for
         *         non-canonical reads.
         */
        short GetVMin() const ;

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds.
         */
        void SetVTime(short vtime) ;

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime() const ;

        /**
         * @brief Sets the DTR line to the specified value.
         * @param dtrState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetDTR(bool dtrState = true) ;

        /**
         * @brief Gets the status of the DTR line.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDTR() const ;

        /**
         * @brief Set the RTS line to the specified value.
         * @param rtsState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetRTS(bool rtsState = true) ;

        /**
         * @brief Get the status of the RTS line.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS() const ;

        /**
         * @brief Get the status of the CTS line.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCTS() ;

        /**
         * @brief Get the status of the DSR line.
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

    protected:

        /**
         * @brief Performs an operation that is defined separately for each
         *        class derived from streambuf. The default behavior is to
         *        do nothing if gptr() is non-null and gptr()!=egptr().
         *        Also, setbuf(0, 0) usually means unbuffered I/O and
         *        setbuf(p, n) means use p[0]...p[n-1] to hold the buffered
         *        characters. In general, this method implements the
         *        subclass's notion of getting memory for the buffered
         *        characters. 
         *
         *        In the case of SerialStreamBuffer, we want to keep using
         *        unbuffered I/O. Hence, using this method has no effect at
         *        present.
         *
         *
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns a pointer to this streambuf object.
         *
         */
        virtual std::streambuf* setbuf(char_type* character, 
                                       std::streamsize numberOfBytes) override ;

        /**
         * @brief Writes up to n characters from the character sequence at 
         *        char s to the serial port associated with the buffer.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters that were successfully
         *         written to the serial port. 
         */
        virtual std::streamsize xsputn(const char_type* character, 
                                       std::streamsize numberOfBytes) override ;
           
        /**
         * @brief Reads up to n characters from the serial port and returns
         *        them through the character array located at s.
         * @param character Pointer to the character buffer to write to the serial port.
         * @param numberOfBytes The number of characters to write to the serial port.
         * @return Returns the number of characters actually read from the
         *         serial port. 
         */
        virtual std::streamsize xsgetn(char_type* character, 
                                       std::streamsize numberOfBytes) override ;

        /**
         * @brief Writes the specified character to the associated serial port.
         * @param character The character to be written to the serial port.
         * @return Returns the character. 
         */
        virtual int_type overflow(const int_type character) override ;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for buffered I/O while uflow() is called
         *        for unbuffered I/O.
         * @return Returns the next character from the serial port.
         */
        virtual int_type underflow() override ;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for unbuffered I/O while underflow() is
         *        called for unbuffered I/O.
         * @return Returns the next character from the serial port.  
         */
        virtual int_type uflow() override ;

        /**
         * @brief This function is called when a putback of a character
         *        fails. This must be implemented for unbuffered I/O as all
         *        streambuf subclasses are required to provide putback of
         *        at least one character.
         * @param character The character to putback.
         * @return Returns The character iff successful, otherwise eof to signal an error.
         */
        virtual int_type pbackfail(const int_type character) override ;

        /**
         * @brief Checks whether input is available on the port.
         *        If you call \c SerialStream::in_avail, this method will
         *        be called to check for available input.
         *        \code
         *        while(serial_port.rdbuf()->in_avail() > 0)
         *        {
         *            serial_port.get(ch) ;
         *            ...
         *        }
         *        \endcode
         * @return Returns 1 if characters are available at the serial port,
         *         0 if no characters are available, and -1 if unsuccessful.
         */
        virtual std::streamsize showmanyc() override ;

    private:

        /**
         * @brief Forward declaration of the Implementation class folowing
         *        the PImpl idiom.
         */
        class Implementation;

        /**
         * @brief Pointer to Implementation class instance.
         */
        std::unique_ptr<Implementation> mImpl;

    } ; // class SerialStreamBuf

} // namespace LibSerial
