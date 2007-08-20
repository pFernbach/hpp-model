# 
#  Copyright 
# 


noinst_PROGRAMS = testhppDevice
testhppDevice_SOURCES = main.cpp \
	hppDeviceTest.cpp

testhppDevice_CPPFLAGS = \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(ROBOTDYNAMICSIMPL_CFLAGS)$ \
	$(KINEOCONTROLLER_CFLAGS)$
testhppDevice_LDFLAGS = \
	$(ROBOTDYNAMICSIMPL_LIBS)$ \
	$(KINEOCONTROLLER_LIBS)$ \
	../src/.libs/libhppModel.a
