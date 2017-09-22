ADD_EXECUTABLE(SerialPortReadExample
  serial_port_read.cpp
)

TARGET_LINK_LIBRARIES(SerialPortReadExample
  libserial_static
)

ADD_EXECUTABLE(SerialPortWriteExample
  serial_port_write.cpp
)

TARGET_LINK_LIBRARIES(SerialPortWriteExample
  libserial_static
)

ADD_EXECUTABLE(SerialStreamReadExample
  serial_stream_read.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamReadExample
  libserial_static
)

ADD_EXECUTABLE(SerialStreamWriteExample
  serial_stream_write.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamWriteExample
  libserial_static
)
