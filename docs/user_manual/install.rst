Install
=======

To install LibSerial the current release package on many Linux distributions you may simply use the package manager associated with your distribution:

For Debian distrbutions:

.. code-block:: c++

   sudo apt install libserial-dev

For Arch Linux distributions:

.. code-block:: c++

   sudo pacman -S libserial-dev

To install LibSerial from source, first clone the repository at https://github.com/crayzeewulf/libserial

Using https:

.. code-block:: bash

   git clone https://github.com/crayzeewulf/libserial.git

Using ssh:

.. code-block:: bash

   git clone git@github.com:crayzeewulf/libserial.git

Next, using make, execute the following commands from your libserial directory:

.. code-block:: c++

   make -F Makefile.dist
   ./configure
   make

To install the build to your /usr/local/ directory your may simply:

.. code-block:: c++

   sudo make install

The code is also easily built using cmake via a bash script:

.. code-block:: c++

   ./compile.sh