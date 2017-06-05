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

#include <termios.h>


using namespace LibSerial ;
using namespace std ;

SerialStream::SerialStream() : 
    std::iostream(0), mIOBuffer(0) 
{
    //
    // Close the stream
    //
    Close() ;
}


SerialStream::~SerialStream() 
{
    // 
    // If a SerialStreamBuf is associated with this SerialStream
    // then we need to destroy it here.
    //
    if ( mIOBuffer )
    {
        delete mIOBuffer ;
    }
}


void 
SerialStream::Close() 
{
    //
    // If a SerialStreamBuf is associated with the SerialStream then
    // destroy it.
    //
    if ( mIOBuffer )
    {
        delete mIOBuffer ;
        mIOBuffer = 0 ;
    }
}

bool
SerialStream::IsOpen() const 
{
    //
    // Checks to see if mIOBuffer is a null buffer, if not,
    // calls the is_open() function on this streams SerialStreamBuf,
    // mIOBuffer
    //
    if ( ! mIOBuffer )
    {
        return false ;
    }
    return mIOBuffer->is_open() ;
}

SerialStream::SerialStream( const string       fileName, 
                            ios_base::openmode openMode ) :
    iostream(0),
    mIOBuffer(0) 
{
    this->Open( fileName, openMode ) ;
    return ;
}

SerialStream::SerialStream( const std::string fileName,
                            const SerialStreamBuf::BaudRateEnum baudRate,
                            const SerialStreamBuf::CharSizeEnum charSize,
                            const SerialStreamBuf::ParityEnum parityType,
                            const short numOfStopBits,
                            const SerialStreamBuf::FlowControlEnum flowControlType ) :
    iostream(0),
    mIOBuffer(0)
{
    this->Open( fileName ) ;
    this->SetBaudRate( baudRate ) ;
    this->SetCharSize( charSize ) ;
    this->SetParity( parityType ) ;
    this->SetNumOfStopBits( numOfStopBits ) ;
    this->SetFlowControl( flowControlType ) ;
    return ;
}

void 
SerialStream::Open( const std::string       fileName, 
                    std::ios_base::openmode openMode ) 
{
    //
    // Create a new SerialStreamBuf if one does not exist. 
    //
    if ( ! mIOBuffer )
    {
        mIOBuffer = new SerialStreamBuf ;
        assert( 0 != mIOBuffer ) ;
        this->rdbuf( mIOBuffer ) ;
    }
    //
    // Open the serial port. 
    //
    if ( 0 == mIOBuffer->open(fileName, openMode) )
    {
        setstate(badbit) ;    
    }
    return ;
}

void 
SerialStream::SetBaudRate( 
    const SerialStreamBuf::BaudRateEnum baudRate ) 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer ) 
    {
        //
        // Try to set the baud rate. If the corresponding function of
        // the SerialStreamBuf class returns BAUD_INVALID, then we
        // have a problem and the stream is no longer valid for I/O.
        //
        if ( SerialStreamBuf::BAUD_INVALID == 
            my_buffer->SetBaudRate(baudRate) ) 
        {
            setstate(badbit) ;
        }
    } 
    else 
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer
        // of class other than SerialStreamBuf. In either case, we
        // have a problem and we should stop all I/O using this
        // stream.
        //
        setstate(badbit) ;
    }
    return ;
}

SerialStreamBuf::BaudRateEnum 
SerialStream::BaudRate() 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer ) 
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->BaudRate() ;
    } 
    else 
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
        return SerialStreamBuf::BAUD_INVALID ;
    }
}

void
SerialStream::SetCharSize( 
    const SerialStreamBuf::CharSizeEnum charSize ) 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer ) 
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if ( SerialStreamBuf::CHAR_SIZE_INVALID == 
             my_buffer->SetCharSize(charSize) ) 
        {
            setstate(badbit) ;
        }
    } 
    else 
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
    }
    return ;
}

SerialStreamBuf::CharSizeEnum
SerialStream::CharSize() 
{
    SerialStreamBuf* my_buffer = 
        dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer ) 
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->CharSize() ;
    } 
    else 
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
        return SerialStreamBuf::CHAR_SIZE_INVALID ;
    }
}

void
SerialStream::SetNumOfStopBits(short stop_bits)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if ( -1 == my_buffer->SetNumOfStopBits(stop_bits) )
        {
            setstate(badbit) ;
        }
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
    }
    return ;
}

short
SerialStream::NumOfStopBits()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->NumOfStopBits() ;
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
        return -1 ;
    }
}

void 
SerialStream::SetParity(const SerialStreamBuf::ParityEnum parity)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if ( SerialStreamBuf::PARITY_INVALID == my_buffer->SetParity(parity) )
        {
            setstate(badbit) ;
        }
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
    }
    return ;
}

SerialStreamBuf::ParityEnum
SerialStream::Parity()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->Parity() ;
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
        return SerialStreamBuf::PARITY_INVALID ;
    }
}

void 
SerialStream::SetFlowControl(const SerialStreamBuf::FlowControlEnum flow_c)
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if ( SerialStreamBuf::FLOW_CONTROL_INVALID == my_buffer->SetFlowControl(flow_c) )
        {
            setstate(badbit) ;
        }
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
    }
    return ;
}

short
SerialStream::SetVMin( short vmin )
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    if ( my_buffer )
    {
        if ( -1 == my_buffer->SetVMin( vmin ) )
        {
            setstate(badbit) ;
            return -1;
        }
    }
    else
    {
        setstate(badbit) ;
        return -1 ;
    }
    return vmin;
}

short
SerialStream::VMin()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    if ( my_buffer )
    {
        return my_buffer->VMin() ;
    }
    else
    {
        setstate(badbit) ;
        return -1 ;
    }
}

short
SerialStream::SetVTime( short vtime )
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    if ( my_buffer )
    {
        if ( -1 == my_buffer->SetVTime( vtime ) )
        {
            setstate(badbit) ;
            return -1 ;
        }
    }
    else
    {
        setstate(badbit) ;
        return -1 ;
    }

    return vtime ;
}

short
SerialStream::VTime()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    if ( my_buffer )
    {
        return my_buffer->VTime() ;
    }
    else
    {
        setstate(badbit) ;
        return -1 ;
    }
}

SerialStreamBuf::FlowControlEnum
SerialStream::FlowControl()
{
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if ( my_buffer )
    {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->FlowControl() ;
    }
    else
    {
        //
        // If the dynamic_cast above failed then we either have a NULL
        // streambuf associated with this stream or we have a buffer of
        // class other than SerialStreamBuf. In either case, we have a
        // problem and we should stop all I/O using this stream.
        //
        setstate(badbit) ;
        return SerialStreamBuf::FLOW_CONTROL_INVALID ;
    }
}
