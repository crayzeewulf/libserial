/***************************************************************************
 *   @file SerialStream.h                                                  *
 *   @copyright (C) 2004 by Manish Pagey                                      *
 *   crayzeewulf@users.sourceforge.net                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _SerialStream_h_
#define _SerialStream_h_

#include "SerialStreamBuf.h"
#include "SerialPortConstants.h"


namespace LibSerial 
{
    /**
     * @brief A stream class for accessing serial ports on POSIX operating
     *        systems. A lot of the functionality of this class has been
     *        obtained by looking at the code of libserial package by Linas
     *        Vepstas, (linas@linas.org) and the excellent document on
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
         * @brief Constructor that takes a filename and an openmode to
         *        construct a SerialStream object. This results in a
         *        call to basic_fstream::open(s,mode). This is the only
         *        way to contruct an object of this class. We have to
         *        enforce this instead of providing a default
         *        constructor because we want to get a file descriptor
         *        whenever the basic_fstream::open() function is
         *        called. However, this function is not made virtual in
         *        the STL hence it is probably not very safe to overload
         *        it. We may decide to overload it later but the users
         *        of this class will have to make sure that this class
         *        is not used as an fstream class. The SerialStream will
         *        be in the "open" state (same state as after calling
         *        the Open() method) after calling this constructor.
         *
         *        If the constructor has problems opening the serial port or
         *        getting the file-descriptor for the port, it will set the
         *        failbit for the stream. So, one must make sure that the
         *        stream is in a good state before using it for any further
         *        I/O operations.
         *
         * @param fileName The filename of the serial port. 
         * @param openMode The openmode for the serial port file. 
         *
         */
        explicit SerialStream(const std::string& fileName, 
                              std::ios_base::openmode openMode = std::ios::in|std::ios::out);

        /**
         * @brief Constructor that allows one to create a SerialStream
         *        instance and also initialize the corresponding serial
         *        port with the specified parameters. This was suggested
         *        by Witek Adamus (wit3k). 
         *
         * @note See https://sourceforge.net/tracker/index.php?func=detail&aid=2137885&group_id=9432&atid=359432
         *
         * @param fileName The file descriptor of the serial stream object.
         * @param baudRate The communications baud rate.
         * @param characterSize The size of the character buffer for
         *        storing read/write streams.
         * @param parityType The parity type for the serial stream object.
         * @param numberOfStopBits The number of stop bits.
         * @param flowControlType Flow control for the serial data stream.
         */
        SerialStream(const std::string&   fileName,
                     const BaudRate&      baudRate        = BaudRate::BAUD_DEFAULT,
                     const CharacterSize& characterSize   = CharacterSize::CHAR_SIZE_DEFAULT,
                     const FlowControl&   flowControlType = FlowControl::FLOW_CONTROL_DEFAULT,
                     const Parity&        parityType      = Parity::PARITY_DEFAULT,
                     const StopBits&      stopBits        = StopBits::STOP_BITS_DEFAULT);

        /**
         * @brief Default Destructor.
         *        Closes the stream associated with mFileDescriptor.
         *        Remaining actions are accomplished by the fstream destructor.
         */
        virtual ~SerialStream(); 

        /**
         * @brief Opens the serial port associated with the specified
         *        filename, and the specified mode, mode.
         * @param fileName The file descriptor of the serial stream object.
         * @param openMode The communication mode status when the serial
         *        communication port is opened.
         */
        void Open(const std::string& fileName, 
                  std::ios_base::openmode openMode = std::ios_base::in | std::ios_base::out);

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
         * @brief Sets the input and output baud ratesfor the
         *        Serial Stream object. 
         */
        void SetBaudRate(const BaudRate& baudRate);

        /**
         * @brief Gets the current baud rate being used for serial
         *        communication. This routine queries the serial port for
         *        its current settings and returns the baud rate that is
         *        being used by the serial port.
         * @note This is not a constant function because it checks to see
         *       that it is dealing with a SerialStream with a non-null
         *       buffer. If the buffer is null, it attempts to set the
         *       state of the stream accordingly.
         * @return Returns the current baud rate for the serial port.
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
         * @brief Returns the current flow control setting.
         * @return Returns the current flow control setting.
         */
        FlowControl GetFlowControl();

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity& parityType);

        /**
         * @brief Get the current parity setting for the serial port. 
         * @return Returns the parity setting for the serial port. 
         */
        Parity GetParity();

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param numberOfStopBits The number of stop bits to set. 
         */
        void SetNumberOfStopBits(const StopBits& numberOfStopBits);

        /**
         * @brief Gets the number of stop bits being used during serial communication.
         * @return Returns the number of stop bits.
         */
        StopBits GetNumberOfStopBits(); 

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @param vMin the number of minimum characters to be set.
         */
        void SetVMin(const short& vmin);

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
         */
        void SetVTime(const short& vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();

    protected:

    private:

        /**
         * @brief The copy constructor and the assignment operator are
         *        declared but never defined. This allows the compiler
         *        to catch any attempts to copy instances of this class.
         */
        SerialStream(const SerialStream&);
        SerialStream& operator=(const SerialStream&);

        /**
         * @brief The SerialStreamBuffer object that will be used by the
         *        stream to communicate with the serial port.
         */
        SerialStreamBuf *mIOBuffer;

    }; // class SerialStream

} // namespace LibSerial

#endif // #ifndef _SerialStream_h_