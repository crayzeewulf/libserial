ADD_EXECUTABLE(readPortExample
  read_port.cpp
)

TARGET_LINK_LIBRARIES(readPortExample
  LibSerial
)

ADD_EXECUTABLE(writePortExample
  write_port.cpp
)

TARGET_LINK_LIBRARIES(writePortExample
  LibSerial
)
