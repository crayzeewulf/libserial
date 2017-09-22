ADD_EXECUTABLE(readPortExample
  read_port.cpp
)

TARGET_LINK_LIBRARIES(readPortExample
  libserial_static
)

ADD_EXECUTABLE(writePortExample
  write_port.cpp
)

TARGET_LINK_LIBRARIES(writePortExample
  libserial_static
)
