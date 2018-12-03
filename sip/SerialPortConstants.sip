namespace LibSerial
{

%TypeHeaderCode
#include <libserial/SerialPortConstants.h>
%End
    
    typedef std::vector<unsigned char> DataBuffer;
    
    namespace BaudRate 
    {
        enum
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
            BAUD_DEFAULT = BAUD_115200
        } ;
    };

    namespace CharacterSize
    {
        enum
        {
            CHAR_SIZE_5       = CS5, // !< 5 bit characters.
            CHAR_SIZE_6       = CS6, // !< 6 bit characters.
            CHAR_SIZE_7       = CS7, // !< 7 bit characters.
            CHAR_SIZE_8       = CS8, // !< 8 bit characters.
            CHAR_SIZE_DEFAULT = CS8  // !< 8 bit characters.
        } ;
    };

    namespace FlowControl
    {
        enum
        {
            FLOW_CONTROL_HARDWARE,
            FLOW_CONTROL_SOFTWARE,
            FLOW_CONTROL_NONE,
            FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
        };
    };

    namespace Parity
    {
        enum
        {
            PARITY_EVEN,                    // Even parity.
            PARITY_ODD,                     // Odd parity.
            PARITY_NONE,                    // No parity i.e. parity checking disabled.
            PARITY_DEFAULT = PARITY_NONE	// No parity i.e. parity checking disabled.
        };
    };

    namespace StopBits
    {
        enum
        {
            STOP_BITS_1,                    // 1 stop bit.
            STOP_BITS_2,                    // 2 stop bits.
            STOP_BITS_DEFAULT = STOP_BITS_1	// 1 stop bit.
        };
    };

}; // namespace LibSerial
