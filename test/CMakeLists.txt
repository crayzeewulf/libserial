ADD_EXECUTABLE(UnitTests
  UnitTests.cpp
  )

TARGET_LINK_LIBRARIES(UnitTests
  libserial_static
  GTestMain
)
