/******************************************************************************
 *   @file SerialStream.cc                                                    *
 *   @copyright                                                               *
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
 
#include "SerialStream.h"

#include <cassert>
#include <termios.h>


using namespace LibSerial;

SerialStream::SerialStream()
    : std::iostream(0), mIOBuffer(0) 
{
    // Close the stream
    Close();
}


SerialStream::~SerialStream() 
{
    // If a SerialStreamBuf is associated with this SerialStream
    // then we need to destroy it here.
    if (mIOBuffer)
    {
        delete mIOBuffer;
    }
}

SerialStream::SerialStream(const std::string& fileName, 
                           ios_base::openmode openMode)
    : std::iostream(0)
    , mIOBuffer(0) 
{
    this->Open(fileName, openMode);
    return;
}

SerialStream::SerialStream(const std::string&   fileName,
                           const BaudRate&      baudRate,
                           const CharacterSize& characterSize,
                           const FlowControl&   flowControlType,
                           const Parity&        parityType,
                           const StopBits&      stopBits)
    : std::iostream(0)
    , mIOBuffer(0)
{
    this->Open(fileName);
    this->SetBaudRate(baudRate);
    this->SetCharacterSize(characterSize);
    this->SetFlowControl(flowControlType);
    this->SetParity(parityType);
    this->SetNumberOfStopBits(stopBits);
    return;
}

void
SerialStream::Open(const std::string& fileName, 
                   std::ios_base::openmode openMode) 
{
    // Create a new SerialStreamBuf if one does not exist. 
    if (! mIOBuffer)
    {
        mIOBuffer = new SerialStreamBuf;
        assert(0 != mIOBuffer);
        this->rdbuf(mIOBuffer);
    }

    // Open the serial port. 
    if (0 == mIOBuffer->Open(fileName, openMode))
    {
        setstate(badbit);    
    }

    return;
}

void 
SerialStream::Close() 
{
    // If a SerialStreamBuf is associated with the SerialStream then
    // destroy it.
    if (mIOBuffer)
    {
        delete mIOBuffer;
        mIOBuffer = 0;
    }
}

bool
SerialStream::IsOpen() 
{
    // Checks to see if mIOBuffer is a null buffer, if not, calls
    // the IsOpen() function on this streams SerialStreamBuf mIOBuffer
    if (! mIOBuffer)
    {
        return false;
    }

    return mIOBuffer->IsOpen();
}

void 
SerialStream::SetBaudRate(const BaudRate& baudRate) 
{
    SerialStreamBuf* my_buffer = 
    dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer) 
    {
        // Try to set the baud rate with the corresponding function of
        // the SerialStreamBuf class.
        my_buffer->SetBaudRate(baudRate);
    } 
    else 
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer
        // of class other than SerialStreamBuf. In either case, we
        // have a problem and we should stop all I/O using this stream.
        setstate(badbit);
    }

    return;
}

BaudRate
SerialStream::GetBaudRate() 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer) 
    {
        // Try to get the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        return my_buffer->GetBaudRate();
    } 
    else 
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
        return BaudRate::BAUD_INVALID;
    }
}

void
SerialStream::SetCharacterSize(const CharacterSize& characterSize) 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer) 
    {
        // Try to set the character size with the corresponding function of
        // the SerialStreamBuf class.
        my_buffer->SetCharacterSize(characterSize);
    } 
    else 
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
    }

    return;
}

CharacterSize
SerialStream::GetCharacterSize() 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer) 
    {
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        return my_buffer->GetCharacterSize();
    } 
    else 
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
        return CharacterSize::CHAR_SIZE_INVALID;
    }
}

void
SerialStream::SetFlowControl(const FlowControl& flowControlType)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the flow control with the corresponding function of
        // the SerialStreamBuf class.
        my_buffer->SetFlowControl(flowControlType);
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
    }

    return;
}

FlowControl
SerialStream::GetFlowControl()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        return my_buffer->GetFlowControl();
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
        return FlowControl::FLOW_CONTROL_INVALID;
    }
}

void
SerialStream::SetParity(const Parity& parityType)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the parity type with the corresponding function of
        // the SerialStreamBuf class.
        my_buffer->SetParity(parityType);
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
    }

    return;
}

Parity
SerialStream::GetParity()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        return my_buffer->GetParity();
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
        return Parity::PARITY_INVALID;
    }
}

void
SerialStream::SetNumberOfStopBits(const StopBits& numberOfStopBits)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the number of stop bits with the corresponding function of
        // the SerialStreamBuf class.
        my_buffer->SetNumberOfStopBits(numberOfStopBits);
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
    }

    return;
}

StopBits
SerialStream::GetNumberOfStopBits()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());

    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    if (my_buffer)
    {
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        return my_buffer->GetNumberOfStopBits();
    }
    else
    {
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        setstate(badbit);
        return StopBits::STOP_BITS_INVALID;
    }
}

void
SerialStream::SetVMin(const short& vmin)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());
    
    if (my_buffer)
    {
        my_buffer->SetVMin(vmin);
    }
    else
    {
        setstate(badbit);
    }

    return;
}

short
SerialStream::GetVMin()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());
    
    if (my_buffer)
    {
        return my_buffer->GetVMin();
    }
    else
    {
        setstate(badbit);
        return -1;
    }
}

void
SerialStream::SetVTime(const short& vtime)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());
    
    if (my_buffer)
    {
        my_buffer->SetVTime(vtime);
    }
    else
    {
        setstate(badbit);
    }

    return;
}

short
SerialStream::GetVTime()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf());
    
    if (my_buffer)
    {
        return my_buffer->GetVTime();
    }
    else
    {
        setstate(badbit);
        return -1;
    }
}
