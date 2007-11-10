SUBDIRS=UML 

if HAVE_DOCBOOK2PDF
noinst_DATA = UserManual.pdf
else
noinst_DATA = 
endif

EXTRA_DIST = UserManual.sgml UserManual.dsl

CLEANFILES = UserManual.pdf

UserManual.pdf: UserManual.sgml UserManual.dsl
	$(DOCBOOK2PDF) -d UserManual.dsl UserManual.sgml

