# Libserial

----
Thanks for checking out LibSerial!  LibSerial provides a convenient, object oriented approach to accessing serial ports on POSIX systems.

After you get to know LibSerial a bit, if you find that you have ideas for improvement, please be sure to let us know!

If you simply want to use LibSerial and you already utilize a Debian Linux distribution, use apt to install the current release package:

```sh
sudo apt install libserial-dev
```

Example code to demonstrate how to use the library can be found in the [`examples`](https://github.com/crayzeewulf/libserial/tree/master/examples) directory.
An self-contained example project demonstrating the use of CMake and GNU Autotools (make) can be found in [`examples/example_project`](https://github.com/crayzeewulf/libserial/tree/master/examples/example_project) directory.

## Developers

If you are a developer and would like to make use of the latest code, you will need to have a few packages installed to build LibSerial:
	a recent g++ release, (anything after gcc-3.2 should work), the python sip library, the boost unit test library, and Google Test (gtest).  For Debian users:

```sh
sudo apt update
sudo apt install autogen autoconf build-essential cmake graphviz libboost-dev libgtest-dev libtool python-sip-dev doxygen
```
----
If you get the source code from github and would like to install the library, there are a few steps you will need to accomplish:


----
If you are using CMake you can simply run the `compile.sh` script:
```sh
./compile.sh
```

----
If you are using GNU Autotools (make):

GNU Autotools is currently configured to built all unit tests, so first you will need to compile the GTest library object files and copy `libgtest.a` and `libgtest_main.a` into your `/usr/lib/` directory which you can accomplish by running the `build-gtest` convenience script:
```sh
./gtest.sh
```

To generate the configure script:

```sh
make -f Makefile.dist
```

----
To execute the `configure` script, first create a build directory, then run the script from the build directory as follows:

```sh
./configure 
```

You can specify an installation directory different from the default, (/usr/local/), by adding `--prefix=/installation/directory/path/` to the configure command.  For example, to install into the top level include directory as the package manager would accomplish, you can simply run the following:
```sh
./configure --prefix=/usr/
```

Once you have executed the `configure` script, you can build the library with `make` and install with `make install`:

```sh
make
sudo make install
```

----
If you are interested in running the unit tests, ensure serial port names are appropriate for your hardware configuration in the `test/UnitTests.cpp` file:

```cpp
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"
```

The unit tests will be built during the make steps above or by running the cmake compile script:

```sh
./compile.sh
```

Unit test executables built using make can be run from the `build` directory using the command:
```sh
ctest -V .
```

Alternatively, unit test executables built using CMake can be run from the libserial/build/bin/ directory: 
```sh
./build/bin/UnitTests
./build/bin/unit_tests
```

----
Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).

----
(Let us know that this repository was useful to you by clicking the "star" in the upper right corner of the LibSerial Github home page!)
