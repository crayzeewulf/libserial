# Libserial

Thanks for checking out LibSerial!  LibSerial provides a convenient, object oriented approach to accessing serial ports on Linux operating system.

After you get to know LibSerial a bit, if you find that you have ideas for improvement, please be sure to let us know!

If you simply want to use LibSerial and you already utilize a Debian Linux distribution, use apt to install the current release package:

```sh
sudo apt install libserial-dev
```

Example code to demonstrate how to use the library can be found in the [`examples`](https://github.com/crayzeewulf/libserial/tree/master/examples) directory.
An self-contained example project demonstrating the use of CMake and GNU Autotools (make) can be found in [`examples/example_project`](https://github.com/crayzeewulf/libserial/tree/master/examples/example_project) directory.

## Developers

If you are a developer and would like to make use of the latest code, you will
need to have a few packages installed to build LibSerial: a recent g++ release
(anything after gcc-3.2 should work), autotools, cmake, doxygen, sphinx, the
python sip library, the boost unit test library, pkg-config, and Google Test
(gtest). The following commands should install the required packages for
Debian/Ubuntu users:

```sh
sudo apt update
sudo apt install g++ git autogen autoconf build-essential cmake graphviz \
                 libboost-dev libboost-test-dev libgtest-dev libtool \
                 python-sip-dev doxygen python-sphinx pkg-config \
                 python3-sphinx-rtd-theme
```


If you get the source code from github and would like to install the library, there are a few steps you will need to accomplish:

### Building Using CMake

If you are using CMake, to build the library you can simply run the `compile.sh` script:

```sh
./compile.sh
```

To install the library:

```sh
cd build
sudo make install
```

You can specify an installation directory different from the default, (/usr/local/), by replacing the `cmake ..` command in the `compile.sh` script.  For example, to install under `/usr` instead of the `/usr/local` directory, use the following:

```sh
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
```

### Building Using GNU Autotools

GNU Autotools is currently configured to built all unit tests, so first you will need to compile the GTest library object files and copy `libgtest.a` and `libgtest_main.a` into your `/usr/lib/` directory which you can accomplish by running the `gtest.sh` convenience script:

```sh
./gtest.sh
```

To generate the configure script:

```sh
make -f Makefile.dist
```

To execute the `configure` script, first create a build directory, then run the script from the build directory as follows:

```sh
../configure
```

You can specify an installation directory different from the default, (`/usr/local/`), by adding `--prefix=/installation/directory/path/` to the configure command.  For example, to install into the top level include directory as the package manager would accomplish, you can simply run the following:

```sh
./configure --prefix=/usr/
```

Once you have executed the `configure` script, you can build the library with `make` and install with `make install`:

```sh
make
sudo make install
```

## Example Code and Unit Tests

If you are interested in running the unit tests or example code, ensure serial port names are appropriate for your hardware configuration in the `examples/` directory files and in the `test/UnitTests.h` file as such:

```cpp
constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0" ;
constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB1" ;
```

Example code and Unit test executables are easily built using the cmake compile script and can be run from the `build` directory:

```sh
./compile
```

```sh
./build/bin/UnitTests
```

```sh
./build/bin/SerialPortReadWriteExample
```

Unit test executables built using make can be run from the `build` directory in the following manner:

```sh
ctest -V .
```

## Hardware and Software Considerations

If needed, you can grant user permissions to utilize the hardware ports in the following manner, (afterwards a reboot is required):

```sh
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

## Socat

Socat is a useful tool to allow hardware ports to communicate on the same system via a software pipe.  As an example, to allow hardware UART port `/dev/ttyS0` to communicate via software with hardware UART port `/dev/ttyS1`:

```sh
socat -d -d pty,raw,echo=0,link=/dev/ttyS0 pty,raw,echo=0,link=/dev/ttyS1
```

## Documentation

Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).


> Let us know that this repository was useful to you by clicking the "star" in 
> the upper right corner of the LibSerial Github home page!
