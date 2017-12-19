# Libserial

----
LibSerial provides a convenient, object oriented approach to accessing serial ports on POSIX systems.

You will likely need to install a few packages to build LibSerial: a recent g++ release, (anything after gcc-3.2 should work), the python sip library, the boost unit test library, and Google Test (gtest).  For Debian users:

```
sudo update
sudo apt install build-essential libgtest-dev libboost-dev python-sip-dev
```
----
If you get the source code from github and would like to install the library, you will need to generate the configure script first:

```
make -f Makefile.dist
```

----
Then run `configure`:

```
./configure 
```

You can specify an installation directory different from the default, (/usr/local/), by adding the --prefix= command to the configure command.  For example:
```
./configure --prefix=/usr/include/
```

Once you have the `configure` script, you can build the library with `make` and install with `make install`:

```
make
sudo make install
```

----
If you are a developer interested in utilizing the unit tests, ensure serial port names are appropriate for your hardware configuration in the UnitTests.cpp file:

```
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"
```

The unit tests will be built during the make step above or you can build them by simply by running the compile script (which uses cmake):

```
./compile.sh
```

Unit test executables built using make can be run from the libserial/test/ directory:
```
./test/UnitTests
./unit_tests
```

Alternatively, unit test executables built using the compile script can be run from the libserial/build/bin/ directory: 
```
./build/bin/UnitTests
./build/bin/unit_tests
```

----
Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).

----
(You can let people know that this Repository was useful to you by clicking the "Star" in the upper right of the repository home page!)