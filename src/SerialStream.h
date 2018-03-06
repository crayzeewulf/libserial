/******************************************************************************
 *   @file SerialStream.h                                                     *
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

#ifndef _SerialStream_h_
#define _SerialStream_h_

#include "SerialPortConstants.h"
#include "SerialStreamBuf.h"

namespace LibSerial 
{
    /**
     * @brief SerialStream is a stream class for accessing serial ports on
     *        POSIX operating systems. A lot of the functionality of this class
     *        has been obtained by looking at the code of libserial package by
     *        Linas Vepstas, (linas@linas.org) and the excellent document on
     *        serial programming by Michael R. Sweet. This document can be
     *        found at
     *        <ahref="http://www.easysw.com/~mike/serial/serial.html">
     *        http://www.easysw.com/~mike/serial/serial.html</a>.
     *        The libserial package can be found at
     *        <ahref="http://www.linas.org/serial/">
     *        http://www.linas.org/serial/</a>.
     *        This class allows one to set various parameters of a serial
     *        port and then access it like a simple fstream. (In fact, that
     *        is exactly what it does!) It sets the parameters of the
     *        serial port by maintaining a file descriptor for the port and
     *        uses the basic_fstream functions for the IO. We have not
     *        implemented any file locking yet but it will be added soon.
     *
     *        Make sure you read the documentation of the standard fstream
     *        template before using this class because most of the
     *        functionality is inherited from fstream. Also, a lot of
     *        information about the various system calls used in the
     *        implementation can also be found in the Single Unix
     *        Specification (Version 2). A copy of this document can be
     *        obtained from <a href="http://www.UNIX-systems.org/">
     *        http://www.UNIX-systems.org/</a>. We will refer to this
     *        document as SUS-2.
     */
    class SerialStream : public std::iostream
    {
    public:

        /**
         * @brief Default Contructor.
         *        Creates a new SerialStream object but does not open it.
         *        The Open() method will need to be called explicitly on
         *        the object to communicate with the serial port.
         */
        explicit SerialStream();

        /**
         * @brief Constructor that allows a SerialStream instance to be 
         *        created and opened, initializing the corresponding
         *        serial port with the specified parameters.
         *        Suggested by Witek Adamus (wit3k):
         *        https://sourceforge.net/tracker/index.php?func=detail&aid=2137885&group_id=9432&atid=359432
         *
         * @param fileName The file name of the serial stream.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial stream.
         * @param stopBits The number of stop bits for the serial stream.
         * @param flowControlType The flow control type for the serial stream.
         */
        explicit SerialStream(const std::string&   fileName,
                              const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                              const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                              const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                              const Parity&        parityType      = Parity::PARITY_DEFAULT,
                              const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT);

        /**
         * @brief Default Destructor for a SerialStream object
         *        Closes the stream associated with mFileDescriptor, and
         *        also closes the serial port if open.
         *        Remaining actions are accomplished by the fstream destructor.
         */
        virtual ~SerialStream(); 

        /**
         * @brief Opens the serial port associated with the specified
         *        file name and the specified mode.
         * @param fileName The file name of the serial port.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName,
                  const std::ios_base::openmode& openMode = std::ios_base::in | std::ios_base::out);

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
         * @brief Checks if data is available at the input of the serial port.
         * @return Returns true iff data is available to read.
         */
        bool IsDataAvailable();

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
        void SetFlowControl(const FlowControl& flowControlType);

        /**
         * @brief Gets the current flow control setting.
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
         * @note See VMIN in man termios(3).
         * @param vmin the number of minimum characters to be set.
         */
        void SetVMin(const short vmin);

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for
         *         non-canonical reads.
         */
        short GetVMin();

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds.
         */
        void SetVTime(const short vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

        /**
         * @brief Sets the DTR line to the specified value.
         * @param dtrState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetDTR(const bool dtrState = true);

        /**
         * @brief Gets the status of the DTR line.
         * @return Returns true iff the status of the DTR line is high.
         */
        bool GetDTR();

        /**
         * @brief Set the RTS line to the specified value.
         * @param rtsState The line voltage state to be set,
         *        (true = high, false = low).
         */
        void SetRTS(const bool rtsState = true);

        /**
         * @brief Get the status of the RTS line.
         * @return Returns true iff the status of the RTS line is high.
         */
        bool GetRTS();

        /**
         * @brief Get the status of the CTS line.
         * @return Returns true iff the status of the CTS line is high.
         */
        bool GetCTS();

        /**
         * @brief Get the status of the DSR line.
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


    protected:

    private:

        /**
         * @brief Prevents copying of objects of this class by declaring the copy
         *        constructor private. This method is never defined.
         */
        SerialStream(const SerialStream& otherSerialStream) = delete;

        /**
         * @brief Move construction is disallowed.
         */
        SerialStream(const SerialStream&& otherSerialStream) = delete;

        /**
         * @brief Prevents copying of objects of this class by declaring the
         *        assignment operator private. This method is never defined.
         */
        SerialStream& operator=(const SerialStream& otherSerialStream) = delete;

        /**
         * @brief Move assignment is not allowed.
         */
        SerialStream& operator=(const SerialStream&& otherSerialStream) = delete;

        /**
         * @brief The SerialStreamBuffer object that will be used by the
         *        stream to communicate with the serial port.
         */
        SerialStreamBuf* mIOBuffer;

    }; // class SerialStream

} // namespace LibSerial

#endif // #ifndef _SerialStream_h_