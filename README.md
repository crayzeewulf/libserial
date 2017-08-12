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
If you are a developer, in order to run the unit tests, first ensure serial port names are appropriate for your hardware configuration in LibSerialTest.cpp:

```
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"
```

Next, compile the GTest unit tests by running the compile script:

```
./compile.sh
```

The unit tests can then be executed with the following:

```
./build/bin/LibSerialUnitTests
```

----
Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).