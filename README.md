libserial
=========

This library can be used to access the serial ports on POSIX
systems. You will need the latest gcc release (anything after gcc-3.2
should work) to compile libserial.

If you get the source from github, you need to generate the configure script first:

```
make -f Makefile.dist
```

You can skip this step if you are using a release package (which already contains the `configure` script). Once you have the `configure` script, run the following commands:

```shell
./configure 
make
make install
```

Enjoy, 
CrayzeeWulf
