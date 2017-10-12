ADD_EXECUTABLE(MainPageExample
  main_page_example.cpp
)

TARGET_LINK_LIBRARIES(MainPageExample
  libserial_static
)

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

ADD_EXECUTABLE(SerialPortReadWriteExample
  serial_port_read_write.cpp
)

TARGET_LINK_LIBRARIES(SerialPortReadWriteExample
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

ADD_EXECUTABLE(SerialStreamReadWriteExample
  serial_stream_read_write.cpp
)

TARGET_LINK_LIBRARIES(SerialStreamReadWriteExample
  libserial_static
)
