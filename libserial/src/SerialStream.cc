#ifndef _fcntl_h_INCLUDED_
#    include <fcntl.h>
#    define _fcntl_h_INCLUDED_
#endif

#ifndef _std_cstdio_INCLUDED_
#    include <cstdio>
#    define _std_cstdio_INCLUDED_
#endif

#ifndef _termios_h_INCLUDED_
#    include <termios.h>
#    define _termios_h_INCLUDED_
#endif

#ifndef _std_fstream_INCLUDED_
#    include <fstream>
#    define _std_fstream_INCLUDED_
#endif

#ifndef _std_cassert_INCLUDED_
#    include <cassert>
#    define _std_cassert_INCLUDED_
#endif

#ifndef _SerialStream_h_
#    include "SerialStream.h"
#endif

using namespace LibSerial ;
using namespace std ;

SerialStream::SerialStream(const string filename, ios_base::openmode mode) :
    mIOBuffer(0), iostream(0) {
    Open(filename, mode) ;
    return ;
}


void 
SerialStream::SetBaudRate(const SerialStreamBuf::BaudRateEnum baud_rate) {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if( SerialStreamBuf::BAUD_INVALID == my_buffer->SetBaudRate(baud_rate) ) {
            setstate(badbit) ;
        }
    } else {
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

const SerialStreamBuf::BaudRateEnum 
SerialStream::BaudRate() {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->BaudRate() ;
    } else {
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
SerialStream::SetCharSize(const SerialStreamBuf::CharSizeEnum char_size) {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if( SerialStreamBuf::CHAR_SIZE_INVALID == my_buffer->SetCharSize(char_size) ) {
            setstate(badbit) ;
        }
    } else {
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

const SerialStreamBuf::CharSizeEnum
SerialStream::CharSize() {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->CharSize() ;
    } else {
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
SerialStream::SetNumOfStopBits(short stop_bits) {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if( -1 == my_buffer->SetNumOfStopBits(stop_bits) ) {
            setstate(badbit) ;
        }
    } else {
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

const short
SerialStream::NumOfStopBits() {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->NumOfStopBits() ;
    } else {
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
SerialStream::SetParity(const SerialStreamBuf::ParityEnum parity) {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if( SerialStreamBuf::PARITY_INVALID == my_buffer->SetParity(parity) ) {
            setstate(badbit) ;
        }
    } else {
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

const SerialStreamBuf::ParityEnum
SerialStream::Parity() {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->Parity() ;
    } else {
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
SerialStream::SetFlowControl(const SerialStreamBuf::FlowControlEnum flow_c) {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        if( SerialStreamBuf::FLOW_CONTROL_INVALID == my_buffer->SetFlowControl(flow_c) ) {
            setstate(badbit) ;
        }
    } else {
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

const SerialStreamBuf::FlowControlEnum
SerialStream::FlowControl() {
    SerialStreamBuf* my_buffer = dynamic_cast<SerialStreamBuf *>(this->rdbuf()) ;
    //
    // Make sure that we are dealing with a SerialStreamBuf before
    // proceeding. This check also makes sure that we have a non-NULL
    // buffer associated with this stream.
    //
    if( my_buffer ) {
        //
        // Try to set the baud rate. If the corresponding function of the
        // SerialStreamBuf class returns BAUD_INVALID, then we have a
        // problem and the stream is no longer valid for I/O.
        //
        return my_buffer->FlowControl() ;
    } else {
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

