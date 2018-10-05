#include <SerialStream.h>
#include <SerialStreamBuf.h>

#define BOOST_TEST_MODULE libserial
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( SerialStreamBuf_Constructor_Test ) // NOLINT (cert-err58-cpp)
{
    // 
    // SerialStreamBuf should not be open on default construction
    //
    using LibSerial::SerialStreamBuf ;
    SerialStreamBuf buf ;
    BOOST_CHECK( false == buf.IsOpen() ); // NOLINT (cppcoreguidelines-pro-type-vararg)
}

BOOST_AUTO_TEST_CASE( SerialStream_Constructor_Test ) // NOLINT (cert-err58-cpp)
{
    using LibSerial::SerialStream ;
    //
    // SerialStream should not be open on default construction
    //
    {
        SerialStream serial_stream;
        BOOST_CHECK( false == serial_stream.IsOpen() ); // NOLINT (cppcoreguidelines-pro-type-vararg)
    }
    //
    // Attempting to open a non-existent serial port should leave the 
    // serial stream in non-good state.
    //
    {
        SerialStream serial_stream ;
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        BOOST_CHECK_THROW(
            serial_stream.Open("/dev/some_non_existent_device_hope_it_does_not_exist"),
            LibSerial::OpenFailed
        ) ;
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        BOOST_CHECK(not serial_stream.good()) ;
    }
}

