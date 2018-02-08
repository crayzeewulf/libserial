%MappedType std::string
{
%TypeHeaderCode
#include <string>
#include <iostream>
%End

%ConvertToTypeCode
    
    // If sipIsErr is NULL then we must check the type of the
    // Python object and return the result.
    if ( ! sipIsErr )
    {
        return PyString_Check( sipPy );
    }

    char *str_val = 0 ;
    
    if ( PyArg_Parse( sipPy, "s", &str_val ) )
    {
        std::string* ret_val = new std::string( str_val );
        if ( 0 == ret_val )
        {
            *sipIsErr = 1;
            return 0 ;
        }
        *sipCppPtr = ret_val;
    }
    else
    {
        *sipIsErr = 1 ;
        return 0 ;
    }

    return sipGetState( sipTransferObj );
%End
    
%ConvertFromTypeCode

    // Inspired by QtCore/qstring.cpp from PyQt4 source code.
    PyObject *uobj = 
        PyUnicode_FromUnicode( NULL, sipCpp->length() );

    if ( 0 == uobj )
    {
        return 0;
    }

    Py_UNICODE *pyu = PyUnicode_AS_UNICODE( uobj );

    for( unsigned int i=0; i<sipCpp->length(); ++i, ++pyu )
    {
        *pyu = sipCpp->at(i);
    }

    return uobj;    
%End
};

