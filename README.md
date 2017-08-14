# Libserial

----
LibSerial provides a convenient, object oriented approach to accessing serial ports on POSIX systems.

You will need a recent g++ release, (anything after gcc-3.2 should work), to compile libserial.

----
If you get the source code from github and would like to install the library, you will need to generate the configure script first:

```
make -f Makefile.dist
```

----
You can skip this step if you are using a release package (which already contains the `configure` script). Once you have the `configure` script, run the following commands:

```
./configure 
make
make install
```

----
If you are a developer interested in utilizing the unit tests, ensure serial port names are appropriate for your hardware configuration in LibSerialTest.cpp:

```
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"
```

The unit tests will be built during the make step above and another convenient method we've provided is by running the compile script which uses cmake:

```
./compile.sh
```

The unit tests built using make can be executed from the test directory:
```
./test/LibSerialUnitTests
./unit_tests
```
If cmake or the compile script was used to build the library, unit tests can be executed from the libserial/build/bin directory: 
```
./build/bin/LibSerialUnitTests
./build/bin/unit_tests
```

----
Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).