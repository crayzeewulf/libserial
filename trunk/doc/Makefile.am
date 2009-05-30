SUBDIRS=UML 

if HAVE_DOCBOOK2PDF
noinst_DATA = UserManual.pdf
else
noinst_DATA = 
endif

EXTRA_DIST = UserManual.xml 

CLEANFILES = UserManual.pdf

UserManual.pdf: UserManual.xml 
	$(DOCBOOK2PDF) UserManual.xml

