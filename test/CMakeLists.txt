ADD_EXECUTABLE(LibSerialUnitTests
  LibSerialUnitTests.cpp
  )

TARGET_LINK_LIBRARIES(LibSerialUnitTests
  libSerial
  GTestMain
)

ADD_EXECUTABLE(unit_tests
  unit_tests.cpp
  )

TARGET_LINK_LIBRARIES(unit_tests
  libSerial
  ${Boost_LIBRARIES}
)
