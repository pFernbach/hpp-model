# 
#  Copyright 
# 

SUBDIRS = \
	src \
	include \
	unitTesting

hppModeldir = $(libdir)/pkgconfig
hppModel_DATA = hppModel.pc

install-data-hook: documentation

documentation:
	cd doc; $(MAKE) install
