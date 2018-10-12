Design Documentation
====================

LibSerial's Coding standards
----------------------------

Try to utilize these guidelines if you are contributing the LibSerial as a
developer.  Although we attempt to maintain these standards wherever practical,
on occasion you might still discover a few deviations.

Please familiarize yourselves with `C++ Core Guidelines
<http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines>`_ and try to
follow these guidelines.

LibSerial uses ISO standard C++ based on the C++14 standard. 

Use Doxygen style comments (with @ commands) for every:

   * Class
   * Data Member
   * Function

     * @brief command for every function
     * @param command (if not void) for every parameter
     * @return command (if not void)

   * File

     * @file command, (except @example files)
     * @copyright command

Allman (BSD) indentation style


Classes/Namespace/Structure/Enumeration names: CamelCase
Class methods: CamelCase
Class members: mCamelCase

Arguments to methods/functions: camelCase (lower case first word)


Naming Convention
-----------------
Use CamelCase for Files, Classes, Namespace, Structures, Enumerations, Functions, Procedures, and Member Variables.

Filenames are the name of the class or namespace within -- one class per file.

Classes, Namespaces, Structures, Enumerations, and Functions start with a capitalized letter and are nouns: (e.g. SerialPort, SerialStream, etc.).
Inhertied functions may be exceptions.

Function names are a description of the return value, and Procedure names are a strong verb followed by an object. (See Code Complete 2 §7.6 for the difference between a function and a procedure verb.)

Function arguments start with a lowercase letter and are nouns; (e.g. numberOfBytes, etc.)

Member Variables start with a lowercase letter "m" and are nouns; (e.g. mFileDescriptor, etc.).

Use underscores for non-member functions and local variables, lower case with an underscore to separate words; (e.g. lower_case, short_names). 

Constants and Globals are named identically to variables.

Do not use abbreviations and be as precise and descriptive with naming as possible.

Indentation
-----------

Indentation shall be 4 space characters, not tabs.

Braces shall begin and end on the indentation level.

Namespaces are NOT indented.

Case statements are NOT indented.

Class visibility statements are NOT indented (public, protected, private).

One statement per line -- this includes variable declarations.

Do not put short if() ...; statements on one line.

If the constructor initializers don't fit on a single line, put constructor initializer list items one per line, starting with the comma and aligned with the colon separator.  For example:

.. code-block:: c++

   Class::Class()
       : var1(1)
       , var2(2)
       , var3(3)
   {
   ...
   }

The purpose of this indentation policy, which can feel "incorrect" at times is to ensure that changes are isolated to the minimum number of lines.  Our tools, (compilers, editors, diff viewers, and source code repository), all operate on a line-by-line basis.  When someone makes a change that affects a portion anywhere in the line, the tools consider the entire line changed.  This can lead to nasty issues like complex merge conflicts, or, worse, obscure the developer activity.

Include Headers
---------------
A good practice is to include headers in the order most local to least local and alphabetize your lists to avoid duplications. The purpose for this is to ensure that a proper dependency chain is maintained. As the project grows larger, these compilation failures sometimes can be difficult to identify and resolve.

This means that header files have includes alphabetized in the order:

* project includes
* project dependency includes
* system includes

Source files have the includes in the order:

* definition includes
* project includes
* project dependency includes
* system includes
