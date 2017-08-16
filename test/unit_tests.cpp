#include <SerialStreamBuf.h>
#include <SerialStream.h>

#define BOOST_TEST_MODULE libserial
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( SerialStreamBuf_Constructor_Test )
{
    using namespace LibSerial;
    SerialStreamBuf buf;
    BOOST_CHECK( false == buf.IsOpen() );
}

BOOST_AUTO_TEST_CASE( SerialStream_Constructor_Test )
{
    using namespace LibSerial;
    SerialStream serial_stream;
    BOOST_CHECK( false == serial_stream.IsOpen() );
}
