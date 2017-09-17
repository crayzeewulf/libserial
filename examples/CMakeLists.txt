ADD_EXECUTABLE(SerialPortReadExample
  serial_port_read.cpp
)

TARGET_LINK_LIBRARIES(SerialPortReadExample
  libserial
)

ADD_EXECUTABLE(SerialPortWriteExample
  serial_port_write.cpp
)

TARGET_LINK_LIBRARIES(SerialPortWriteExample
  libserial
)

ADD_EXECUTABLE(SerialStreamReadExample
  serial_stream_read.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamReadExample
  libserial
)

ADD_EXECUTABLE(SerialStreamWriteExample
  serial_stream_write.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamWriteExample
  libserial
)
