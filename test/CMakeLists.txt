ADD_EXECUTABLE(LibSerialTests
  LibSerialTests.cpp
  )

TARGET_LINK_LIBRARIES(LibSerialTests
  libserial
  GTestMain
)

ADD_EXECUTABLE(unit_tests
  unit_tests.cpp
  )

TARGET_LINK_LIBRARIES(unit_tests
  libserial
  ${Boost_LIBRARIES}
)
