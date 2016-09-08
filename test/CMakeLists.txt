ADD_EXECUTABLE(unitTests
  LibSerialTest.cpp
  )

TARGET_LINK_LIBRARIES(unitTests
  LibSerial
  GTestMain
)
