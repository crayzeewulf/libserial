#! /bin/bash
rsync -avP doc/user_manual/_build/html/ libserial:/home/project-web/libserial/htdocs/
rsync -avP doc/html/ libserial:/home/project-web/libserial/htdocs/doxygen/
