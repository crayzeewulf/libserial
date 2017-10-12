Design Documentation
====================

LibSerial's Coding standards
----------------------------

Try to utilize these guidelines if you are contributing the LibSerial as a developer.  Although we attempt to maintain these standards wherever practical, on occasion you might still discover a few deviations.

ISO standard C++ only
C++11
Use Doxygen style comments (with @ commands) for every:

   * Class
   * Data Member
   * Function

     * @brief command for every function
     * @param command (if not void) for every parameter
     * @return command (if not void)

   * File

     * @copyright command

Allman (BSD) indentation style

Naming Convention:
------------------
Use CamelCase for everything, (classes, variables, filenames), with the exception of local variables within a functions, in which case use underscores.

Classes start with a capitalized letter and are nouns.

Variables start with a lowercase letter and are nouns.

Typically, functions start with a lowercase letter and begin with verbs, however, to retain recognizable meaning while avoiding termios functions of the same name, most LibSerial methods begin with an uppercase letter and begin with verbs.  Exceptions to this are inherited/overloaded functions.

To name a function, use a description of the return value. 
To name a procedure, use a strong verb followed by an object. (See Code Complete 2 §7.6 for the difference between a function and a procedure verb.)

Filenames are the name of the class or namespace within.  One class per file.

Constants are named identically to variables.

Globals are named identically to variables.

Member variables are named identically to variables.

Indentation:
------------
Indentation shall be 4 space characters, not tabs.

Braces shall begin and end on the indentation level.

Namespaces are NOT indented.

Case statements are NOT indented.

Class visibility statements are NOT indented (public, protected, private).

One statement per line.  This includes variable declarations.

Do not put short if() ...; statements on one line.

If the constructor initializers don't fit on a single line, put constructor initializer list items one per line, starting with the comma and aligned with the colon separator.  For example:

.. code-block:: c++

   Class::Class()
       : var1(1)
       , var2(2)
   {
   ...
   }

The purpose of this indentation policy, which can feel "incorrect" at times is to ensure that changes are isolated to the minimum number of lines.  Our tools: compilers, editors, diff viewers, and source code repository (git), all operate on a line-by-line basis.  When someone makes a change that affects a portion anywhere in the line, the tools consider the entire line changed.  This can lead to nasty issues like complex merge conflicts, or worse obscure the developer activity.

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
