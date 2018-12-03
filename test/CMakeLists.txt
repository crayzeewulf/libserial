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
  libserial_static
  ${Boost_LIBRARIES}
)

target_include_directories(UnitTests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_include_directories(unit_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)


ADD_TEST(NAME UnitTests COMMAND UnitTests)
