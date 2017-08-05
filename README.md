#Libserial

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
    shell
    ./configure 
    make
    make install
```

----
If you are a developer, to compile and run the unit tests simply run the compile script:

```
    ./compile.sh
```

----
Complete documentation is available [here](http://libserial.readthedocs.io/en/latest/index.html).