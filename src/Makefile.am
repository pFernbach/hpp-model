# 
#  Copyright 2007 LAAS-CNRS
#  Author: Florent Lamiraux
# 

lib_LTLIBRARIES = libhppModel.la
libhppModel_la_SOURCES = \
	hppDevice.cpp \
	hppBody.cpp \
	hppJoint.cpp 

libhppModel_la_LDFLAGS = \
	$(ROBOTDYNAMICSIMPL_LIBS)$ \
	$(KINEOCONTROLLER_LIBS)$ 

libhppModel_la_CPPFLAGS = \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(ROBOTDYNAMICSIMPL_CFLAGS)$ \
	$(KINEOCONTROLLER_CFLAGS)$
