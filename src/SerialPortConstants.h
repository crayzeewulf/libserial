#ifndef SERIALPORTCONSTANTS_H
#define SERIALPORTCONSTANTS_H

#include <limits>
#include <termios.h>

namespace LibSerial
{
    /**
     * The baud rates currently supported by the SUS-2 general terminal
     * interface specification. Note that B0 is not supported because
     * it is not really a baud rate (it causes the modem to hang up
     * i.e. drop DTR). Use the close() method instead.
     */
    enum class BaudRate : speed_t
    {
        BAUD_50      = B50,
        BAUD_75      = B75,
        BAUD_110     = B110,
        BAUD_134     = B134,
        BAUD_150     = B150,
        BAUD_200     = B200,
        BAUD_300     = B300,
        BAUD_600     = B600,
        BAUD_1200    = B1200,
        BAUD_1800    = B1800,
        BAUD_2400    = B2400,
        BAUD_4800    = B4800,
        BAUD_9600    = B9600,
        BAUD_19200   = B19200,
        BAUD_38400   = B38400,
        BAUD_57600   = B57600,
        BAUD_115200  = B115200,
        BAUD_230400  = B230400,
        //
        // Bug#1318912: B460800 is defined on Linux but not on Mac OS
        // X. What about other operating systems ?
        //
#ifdef __linux__
        BAUD_460800 = B460800,
        BAUD_500000 = B500000,
        BAUD_576000 = B576000,
        BAUD_921600 = B921600,
        BAUD_1000000 = B1000000, 
        BAUD_1152000 = B1152000, 
        BAUD_1500000 = B1500000,
        BAUD_2000000 = B2000000,
        BAUD_2500000 = B2500000,
        BAUD_3000000 = B3000000,
        BAUD_3500000 = B3500000,
        BAUD_4000000 = B4000000,
#endif
        BAUD_DEFAULT = BAUD_57600,
        BAUD_INVALID = std::numeric_limits<speed_t>::max()
    } ;

} /* LibSerial */ 

#endif /* end of include guard: SERIALPORTCONSTANTS_H */
