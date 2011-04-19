# 
#  Copyright 2007 LAAS-CNRS
# 


noinst_PROGRAMS = testhppDevice

testhppDevice_SOURCES=\
	main.cpp 		\
	hppDeviceTest.cpp	\
	hppDeviceTest.h

testhppDevice_CPPFLAGS = \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(JRL_DYNAMICS_CFLAGS)$ \
	$(KINEOCONTROLLER_CFLAGS)$\
	$(KWSIO_CFLAGS)
testhppDevice_LDFLAGS = \
	$(JRL_DYNAMICS_LIBS)$ \
	$(KINEOCONTROLLER_LIBS)$ \
	$(KWSIO_LIBS) \
	../src/.libs/libhppModel.a
