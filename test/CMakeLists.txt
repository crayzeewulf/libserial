ADD_EXECUTABLE(LibSerialTests
  LibSerialTests.cpp
  )

TARGET_LINK_LIBRARIES(LibSerialTests
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
