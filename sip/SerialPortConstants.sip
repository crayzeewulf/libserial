namespace LibSerial
{

%TypeHeaderCode
#include <SerialPortConstants.h>
%End
    
    // const std::string ERR_MSG_INVALID_BAUD_RATE      = "Invalid baud rate.";
    // const std::string ERR_MSG_INVALID_CHARACTER_SIZE = "Invalid character size.";
    // const std::string ERR_MSG_INVALID_FLOW_CONTROL   = "Invalid flow control.";
    // const std::string ERR_MSG_INVALID_PARITY         = "Invalid parity setting.";
    // const std::string ERR_MSG_INVALID_STOP_BITS      = "Invalid number of stop bits.";
    // const std::string ERR_MSG_READ_TIMEOUT           = "Read timeout";
    // const std::string ERR_MSG_PORT_ALREADY_OPEN      = "Serial port already open.";
    // const std::string ERR_MSG_PORT_NOT_OPEN          = "Serial port not open.";
    //
    // const int MICROSECONDS_PER_MS  =    1000;
    // const int MILLISECONDS_PER_SEC =    1000;
    // const int MICROSECONDS_PER_SEC = 1000000;
    //
    // static constexpr short VMIN_DEFAULT = 1;
    //
    // static constexpr short VTIME_DEFAULT = 0;
    //
    // static constexpr char CTRL_Q = 0x11;
    //
    // static constexpr char CTRL_S = 0x13;
    //
    // class NotOpen
    // {
    // public:
    //     NotOpen(const std::string& whatArg)
    //         : logic_error(whatArg)
    //     {
    //     }
    // };
    //
    // class AlreadyOpen
    // {
    // public:
    //     AlreadyOpen(const std::string& whatArg)
    //         : logic_error(whatArg)
    //     {
    //     }
    // };
    //
    // class OpenFailed
    // {
    // public:
    //     OpenFailed(const std::string& whatArg)
    //         : runtime_error(whatArg)
    //     {
    //     }
    // };
    //
    // class ReadTimeout
    // {
    // public:
    //     ReadTimeout(const std::string& whatArg)
    //         : runtime_error(whatArg)
    //     {
    //     }
    // };

    
    namespace BaudRate 
    {
        enum {
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
        enum {
            CHAR_SIZE_5       = CS5, // !< 5 bit characters.
            CHAR_SIZE_6       = CS6, // !< 6 bit characters.
            CHAR_SIZE_7       = CS7, // !< 7 bit characters.
            CHAR_SIZE_8       = CS8, // !< 8 bit characters.
            CHAR_SIZE_DEFAULT = CS8  // !< 8 bit characters.
        } ;
    };

    namespace FlowControl
    {
        enum {
            FLOW_CONTROL_HARDWARE,
            FLOW_CONTROL_SOFTWARE,
            FLOW_CONTROL_NONE,
            FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE
        };
    };

    namespace Parity
    {
        enum {
            PARITY_EVEN,                    // Even parity.
            PARITY_ODD,                     // Odd parity.
            PARITY_NONE,                    // No parity i.e. parity checking disabled.
            PARITY_DEFAULT = PARITY_NONE	// No parity i.e. parity checking disabled.
        };
    };

    namespace StopBits
    {
        enum {
            STOP_BITS_1,                    // 1 stop bit.
            STOP_BITS_2,                    // 2 stop bits.
            STOP_BITS_DEFAULT = STOP_BITS_1	// 1 stop bit.
        };
    };

}; // namespace LibSerial
