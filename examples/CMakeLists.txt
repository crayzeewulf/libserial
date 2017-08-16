ADD_EXECUTABLE(serialPortReadExample
  serial_port_read.cpp
)

TARGET_LINK_LIBRARIES(serialPortReadExample
  libSerial
)

ADD_EXECUTABLE(serialPortWriteExample
  serial_port_write.cpp
)

TARGET_LINK_LIBRARIES(serialPortWriteExample
  libSerial
)

ADD_EXECUTABLE(SerialStreamReadExample
  serial_stream_read.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamReadExample
  libSerial
)

ADD_EXECUTABLE(SerialStreamWriteExample
  serial_stream_write.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamWriteExample
  libSerial
)
