#! /usr/bin/env python
import ycm_core

def FlagsForFile( filename, **kwargs ):
    flags = [ '-Wall',
              '-Wextra',
              '-Werror',
              '-std=c++11',
              '-x', 'c++',
              '-I',
              '.',
              '-I',
              'src'
            ]

    return { 'flags': flags,
             'do_cache': True }

