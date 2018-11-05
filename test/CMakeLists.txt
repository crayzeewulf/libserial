ADD_EXECUTABLE(UnitTests
  SerialPortUnitTests.cpp
  SerialStreamUnitTests.cpp
  MultiThreadUnitTests.cpp
  UnitTests.cpp
  )

TARGET_LINK_LIBRARIES(UnitTests
  libserial_static
  GTestMain
)

ADD_EXECUTABLE(unit_tests
  unit_tests.cpp
  )

TARGET_LINK_LIBRARIES(unit_tests
  libserial
  ${Boost_LIBRARIES}
)

ADD_TEST(NAME UnitTests COMMAND UnitTests)
