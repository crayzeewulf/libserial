/******************************************************************************
 * @file SerialPort.h                                                         *
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

#include <libserial/SerialPortConstants.h>

#include <ios>
#include <memory>

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    /**
     * @brief SerialPort allows an object oriented approach to serial port
     *        communication.  A serial port object can be created to
     *        allow opening the port with specified modes and settings.
     *        The SerialPort class also provides Get/Set methods to
     *        access the most commonly utilized parameters associated
     *        with serial port communication.
     *
     */
    class SerialPort
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit SerialPort() ;

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
        explicit SerialPort(const std::string&   fileName,
                            const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                            const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                            const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                            const Parity&        parityType      = Parity::PARITY_DEFAULT,
                            const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT) ;

        /**
         * @brief Default Destructor for a SerialPort object. Closes the
         *        serial port associated with mFileDescriptor if open.
         */
        virtual ~SerialPort() noexcept;

        /**
         * @brief Copy construction is disallowed.
         */
        SerialPort(const SerialPort& otherSerialPort) = delete ;

        /**
         * @brief Move construction is disallowed.
         */
        SerialPort(const SerialPort&& otherSerialPort) = delete ;

        /**
         * @brief Copy assignment is disallowed.
         */
        SerialPort& operator=(const SerialPort& otherSerialPort) = delete ;

        /**
         * @brief Move assignment is disallowed.
         */
        SerialPort& operator=(const SerialPort&& otherSerialPort) = delete ;

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
        void SetVMin(const short vmin) ;

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
        void SetVTime(const short vtime) ;

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
        void SetDTR(const bool dtrState = true) ;

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
        void SetRTS(const bool rtsState = true) ;

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

#ifdef __linux__
        /**
         * @brief Gets a list of available serial ports.
         * @return Returns a std::vector of std::strings with the name of
         *         each available serial port.
         */
        std::vector<std::string> GetAvailableSerialPorts() const ;
#endif

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
        void Read(DataBuffer& dataBuffer,
                  size_t      numberOfBytes = 0,
                  size_t      msTimeout = 0) ;

        /**
         * @brief Reads the specified number of bytes from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout). If msTimeout
         *        is zero, then the method will block until all requested bytes
         *        are received. If numberOfBytes is zero and msTimeout is
         *        non-zero,  the method will continue receiving data for the
         *        specified of milliseconds. If numberOfBytes is zero and
         *        msTimeout is zero, the method will return immediately. In all
         *        cases, any data received remains available in the dataString
         *        on return from this method.
         * @param dataString The string to place data into.
         * @param numberOfBytes The number of bytes to read before returning.
         * @param msTimeout The timeout period in milliseconds.
         */
        void Read(std::string& dataString,
                  size_t       numberOfBytes = 0,
                  size_t       msTimeout = 0) ;

        /**
         * @brief Reads a single byte from the serial port. If no data is
         *        available within the specified number of milliseconds,
         *        (msTimeout), then this method will throw a ReadTimeout
         *        exception. If msTimeout is zero, then this method will
         *        block until data becomes available.
         * @param charBuffer The character read from the serial port.
         * @param msTimeout The timeout period in milliseconds.
         */
        void ReadByte(char&  charBuffer,
                      size_t msTimeout = 0) ;

        /**
         * @brief Reads a single byte from the serial port. If no data is
         *        available within the specified number of milliseconds,
         *        (msTimeout), then this method will throw a ReadTimeout
         *        exception. If msTimeout is zero, then this method will
         *        block until data becomes available.
         * @param charBuffer The character read from the serial port.
         * @param msTimeout The timeout period in milliseconds.
         */
        void ReadByte(unsigned char& charBuffer,
                      size_t         msTimeout = 0) ;

        /**
         * @brief Reads a line of characters from the serial port.
         *        The method will timeout if no data is received in the
         *        specified number of milliseconds (msTimeout).
         *        If msTimeout is 0, then this method will block until a line
         *        terminator is received. In all cases, any data received
         *        remains available in the string on return from this method.
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
         * @param dataString The data string to write to the serial port.
         */
        void Write(const std::string& dataString) ;

        /**
         * @brief Writes a single byte to the serial port.
         * @param charbuffer The byte to write to the serial port.
         */
        void WriteByte(char charbuffer) ;

        /**
         * @brief Writes a single byte to the serial port.
         * @param charbuffer The byte to write to the serial port.
         */
        void WriteByte(unsigned char charbuffer) ;

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

    protected:

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

    } ; // class SerialPort

    /**
     * Type-safe and portable equivalent of TEMP_FAILURE_RETRY macro that is
     * provided gcc. See
     * https://www.gnu.org/software/libc/manual/html_node/Interrupted-Primitives.html
     * and signal(7) man-page for details. The purpose of this function is to
     * repeat system calls that are interrupted and set errno to EINTR.
     * Typically, POSIX applications that use signal handlers must check for
     * EINTR after each library function that can return it in order to try the
     * call again. Often programmers forget to check, which is a common source
     * of error. This may also happen in multi-threaded applications. For
     * example, see
     * https://www.linuxquestions.org/questions/programming-9/problem-in-creating-serial-port-application-using-threads-869149/#post4299011
     *
     * As an example of usage of this function, consider the following use of
     * open() system call:
     *
     * @code{.cpp}
     * const auto fd = open("foo.txt", O_WRONLY | O_APPEND);
     * @endcode
     *
     * If this system call is interrupted by a signal, it will return -1 and
     * errno will be set to EINTR. In order to retry this system call on
     * such interruptions, replace the above call with the following:
     *
     * @code{.cpp}
     * const auto fd = call_with_retry(open, "foo.txt", O_WRONLY | O_APPEND);
     * @endcode
     *
     * @param func
     *     The function to be called and retried on EINTR. This function is
     *     expected to return -1 and set errno to EINTR in case of a system
     *     call interruption.
     *
     * @param args
     *     The arguments to be passed to the function.
     *
     * @return
     *     The value returned by the function after it completes without
     *     interruption that sets errno to EINTR.
     */
    template<typename Fn, typename... Args>
    typename std::result_of<Fn(Args...)>::type
    call_with_retry(Fn func, Args... args)
    {
        using result_type = typename std::result_of<Fn(Args...)>::type ;
        result_type result ;
        do {
            result = func(std::forward<Args>(args)...);
        } while((result == -1) and (errno == EINTR)) ;
        return result ;
    }
} // namespace LibSerial
