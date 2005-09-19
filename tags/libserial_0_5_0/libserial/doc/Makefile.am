SUBDIRS=UML 

noinst_PROGRAMS: UserManual.pdf

UserManual_pdf_SOURCES = UserManual.sgml UserManual.dsl

UserManual.pdf: UserManual.sgml UserManual.dsl
	docbook2pdf -d UserManual.dsl UserManual.sgml

CLEANFILES = UserManual.pdf

