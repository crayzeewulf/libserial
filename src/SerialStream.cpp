/******************************************************************************
 * @file SerialStream.cpp                                                     *
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
#include "SerialStream.h"

#include <cassert>

namespace LibSerial 
{
    SerialStream::SerialStream() : std::iostream(nullptr)
    {
        this->flush() ;
    }

    SerialStream::SerialStream(const std::string&   fileName,
                               const BaudRate&      baudRate,
                               const CharacterSize& characterSize,
                               const FlowControl&   flowControlType,
                               const Parity&        parityType,
                               const StopBits&      stopBits) : 
        std::iostream(nullptr)
    {
        this->Open(fileName) ;  // NOLINT (fuchsia-default-arguments)
        this->SetBaudRate(baudRate) ;
        this->SetCharacterSize(characterSize) ;
        this->SetFlowControl(flowControlType) ;
        this->SetParity(parityType) ;
        this->SetStopBits(stopBits) ;
        this->FlushIOBuffers() ;
    }

    SerialStream::~SerialStream() 
    {
        // Close the serial stream if it is open.
        if (this->IsOpen())
        {
            this->FlushIOBuffers() ;
            this->Close() ;
        }
    }

    void
    SerialStream::Open(const std::string& fileName,
                       const std::ios_base::openmode& openMode)
    try
    {
        // Create a new SerialStreamBuf if one does not exist. 
        if (mIOBuffer == nullptr)
        {
            mIOBuffer = std::make_unique<SerialStreamBuf>() ;
            assert(mIOBuffer != nullptr) ;  // NOLINT (cppcoreguidelines-pro-bounds-array-to-pointer-decay)
            this->rdbuf(mIOBuffer.get()) ;
        }

        // Open the serial port. 
        mIOBuffer->Open(fileName, openMode) ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::Close()
    {
        // If a SerialStreamBuf is associated with this SerialStream
        // we need to destroy it.
        mIOBuffer = nullptr ;
    }

    void
    SerialStream::DrainWriteBuffer()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to flush the input buffers the serial port with the correspoding
            // function of the SerialStreamBuf class.
            my_buffer->DrainWriteBuffer() ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer
            // of class other than SerialStreamBuf. In either case, we
            // have a problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::FlushInputBuffer()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to flush the input buffers the serial port with the correspoding
            // function of the SerialStreamBuf class.
            my_buffer->FlushInputBuffer() ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer
            // of class other than SerialStreamBuf. In either case, we
            // have a problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::FlushOutputBuffer()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to flush the output buffers the serial port with the correspoding
            // function of the SerialStreamBuf class.
            my_buffer->FlushOutputBuffer() ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer
            // of class other than SerialStreamBuf. In either case, we
            // have a problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::FlushIOBuffers()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to flush the I/O buffers the serial port with the correspoding
            // function of the SerialStreamBuf class.
            my_buffer->FlushIOBuffers() ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer
            // of class other than SerialStreamBuf. In either case, we
            // have a problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::IsDataAvailable()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to determine if data is available with the correspoding
            // function of the SerialStreamBuf class.
            return my_buffer->IsDataAvailable() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer
        // of class other than SerialStreamBuf. In either case, we
        // have a problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return false ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::IsOpen()
    try
    {
        // Checks to see if mIOBuffer is a null buffer, if not, calls
        // the IsOpen() function on this stream's SerialStreamBuf mIOBuffer
        if (mIOBuffer == nullptr)
        {
            return false;
        }
        return mIOBuffer->IsOpen() ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void 
    SerialStream::SetBaudRate(const BaudRate& baudRate)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr) 
        {
            // Try to set the baud rate with the corresponding function of
            // the SerialStreamBuf class.
            my_buffer->SetBaudRate(baudRate) ;
        } 
        else 
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer
            // of class other than SerialStreamBuf. In either case, we
            // have a problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    BaudRate
    SerialStream::GetBaudRate()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr) 
        {
            // Try to get the baud rate. If the corresponding function of the
            // SerialStreamBuf class returns BAUD_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            return my_buffer->GetBaudRate() ;
        } 
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return BaudRate::BAUD_INVALID;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetCharacterSize(const CharacterSize& characterSize)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr) 
        {
            // Try to set the character size with the corresponding function of
            // the SerialStreamBuf class.
            my_buffer->SetCharacterSize(characterSize) ;
        } 
        else 
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    CharacterSize
    SerialStream::GetCharacterSize()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr) 
        {
            // Try to get the character size. If the corresponding function of the
            // SerialStreamBuf class returns CHAR_SIZE_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            return my_buffer->GetCharacterSize() ;
        } 
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return CharacterSize::CHAR_SIZE_INVALID;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetFlowControl(const FlowControl& flowControlType)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the flow control. If the corresponding function of the
            // SerialStreamBuf class returns FLOW_CONTROL_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            my_buffer->SetFlowControl(flowControlType) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    FlowControl
    SerialStream::GetFlowControl()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the flow control. If the corresponding function of the
            // SerialStreamBuf class returns FLOW_CONTROL_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            return my_buffer->GetFlowControl() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return FlowControl::FLOW_CONTROL_INVALID;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetParity(const Parity& parityType)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the parity type. If the corresponding function of the
            // SerialStreamBuf class returns PARITY_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            my_buffer->SetParity(parityType) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    Parity
    SerialStream::GetParity()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the parity type. If the corresponding function of the
            // SerialStreamBuf class returns PARITY_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            return my_buffer->GetParity() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return Parity::PARITY_INVALID;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetStopBits(const StopBits& stopBits)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the number of stop bits. If the corresponding function of the
            // SerialStreamBuf class returns STOP_BITS_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            my_buffer->SetStopBits(stopBits) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    StopBits
    SerialStream::GetStopBits()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the number of stop bits. If the corresponding function of the
            // SerialStreamBuf class returns STOP_BITS_INVALID, then we have a
            // problem and the stream is no longer valid for I/O.
            return my_buffer->GetStopBits() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return StopBits::STOP_BITS_INVALID;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetVMin(const short vmin)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the vMin number of characters.
            my_buffer->SetVMin(vmin) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    short
    SerialStream::GetVMin()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the vMin number of characters.
            return my_buffer->GetVMin() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return -1;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetVTime(const short vtime)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the vTime duration in deciseconds.
            my_buffer->SetVTime(vtime) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    short
    SerialStream::GetVTime()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the vTime duration in deciseconds.
            return my_buffer->GetVTime() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return -1;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetDTR(const bool dtrState)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the dtr line state.
            my_buffer->SetDTR(dtrState) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::GetDTR()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the vTime duration in deciseconds.
            return my_buffer->GetDTR() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return false ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    void
    SerialStream::SetRTS(const bool rtsState)
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to set the RTS line state.
            my_buffer->SetRTS(rtsState) ;
        }
        else
        {
            // If the dynamic_cast above failed then we either have a NULL
            // streambuf associated with this stream or we have a buffer of
            // class other than SerialStreamBuf. In either case, we have a
            // problem and we should stop all I/O using this stream.
            setstate(badbit) ;
        }
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::GetRTS()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the RTS line state.
            return my_buffer->GetRTS() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return false ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::GetCTS()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the CTS line state.
            return my_buffer->GetCTS() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return false ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    bool
    SerialStream::GetDSR()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the DSR line state.
            return my_buffer->GetDSR() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return false ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    int
    SerialStream::GetFileDescriptor()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to get the file descriptor.
            return my_buffer->GetFileDescriptor() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return -1;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    int
    SerialStream::GetNumberOfBytesAvailable()
    try
    {
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {
            // Try to determine if data is available with the correspoding
            // function of the SerialStreamBuf class.
            return my_buffer->GetNumberOfBytesAvailable() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer
        // of class other than SerialStreamBuf. In either case, we
        // have a problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return -1 ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }

    std::vector<std::string>
    SerialStream::GetAvailableSerialPorts()
    try
    {   
        auto my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;

        // Make sure that we are dealing with a SerialStreamBuf before
        // proceeding. This check also makes sure that we have a non-NULL
        // buffer associated with this stream.
        if (my_buffer != nullptr)
        {     
            // Try to get the file descriptor.
            return my_buffer->GetAvailableSerialPorts() ;
        }
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit) ;
        return {} ;
    }
    catch (const std::exception&)
    {
        setstate(std::ios_base::failbit) ;
        throw ;
    }
} // namespace LibSerial
