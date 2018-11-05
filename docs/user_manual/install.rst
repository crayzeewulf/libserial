Install
=======

To install LibSerial the current release package on many Linux distributions you may simply use the package manager associated with your distribution:

For Debian distrbutions:

.. code-block:: bash

   sudo apt install libserial-dev

For Arch Linux distributions:

.. code-block:: bash

   sudo pacman -S libserial-dev

To install LibSerial from source, first clone the repository at https://github.com/crayzeewulf/libserial

Using https:

.. code-block:: bash

   git clone https://github.com/crayzeewulf/libserial.git

Using ssh:

.. code-block:: bash

   git clone git@github.com:crayzeewulf/libserial.git

Next, using make, execute the following commands from your libserial directory:

.. code-block:: bash

   make -F Makefile.dist
   ./configure
   make

To install the build to your /usr/local/ directory your may simply:

.. code-block:: bash

   sudo make install

To install to another directory, simply use the *prefix* argument in the configure step above:

.. code-block:: bash

   ./configure --prefix=<DIRECTORY_NAME>

The code is also easily built using `CMake` via a bash script:

.. code-block:: bash

   ./compile.sh

To install, change directories to the build directory and proceed as with make:

.. code-block:: bash

   cd build/
   sudo make install