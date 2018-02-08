# Libserial

----
Thanks for checking out LibSerial!  LibSerial provides a convenient, object oriented approach to accessing serial ports on POSIX systems.

After you get to know LibSerial a bit, if you find that you have ideas for improvement, please be sure to let us know!

If you simply want to use LibSerial and you already utilize a Debian Linux distribution, use apt to install the current release package:

```
sudo apt install libserial-dev
```

Otherwise, if you are a developer and would like to make use of the latest code, you will need to have a few packages installed to build LibSerial:
	a recent g++ release, (anything after gcc-3.2 should work), the python sip library, the boost unit test library, and Google Test (gtest).  For Debian users:

```
sudo apt update
sudo apt install autogen autoconf build-essential cmake graphviz libboost-dev libgtest-dev libtool python-sip-dev doxygen
```
----
If you get the source code from github and would like to install the library, there are a few steps you will need to accomplish:

Fist, compile the GTest library object files and copy libgtest.a and libgtest_main.a into your /usr/lib/ directory:
```
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```

Next, generate the configure script first:

```
make -f Makefile.dist
```

----
Then run `configure`:

```
./configure 
```

You can specify an installation directory different from the default, (/usr/local/), by adding `--prefix=/installation/directory/path/` to the configure command.  For example:
```
./configure --prefix=/usr/include/
```

Once you have the `configure` script, you can build the library with `make` and install with `make install`:

```
make
sudo make install
```

----
If you are interested in running the unit tests, ensure serial port names are appropriate for your hardware configuration in the UnitTests.cpp file:

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
(You can let people know that this Repository was useful to you by clicking the "star" in the upper right of the repository home page!)