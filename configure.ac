dnl
dnl configure.ac file for package hppModel
dnl Copyright 2007-2008 (c) LAAS-CNRS
dnl
dnl Author: Florent Lamiraux
dnl

AC_INIT([hppModel], [1.8.0.99], [hpp@laas.fr])
AC_PREREQ(2.59)

if test "$CFLAGS$CXXFLAGS" = ""; then
  setdefaultflags=yes
else
  setdefaultflags=no
fi

AM_INIT_AUTOMAKE([no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

if test "$setdefaultflags" = "yes"; then
  if test "$GCC" = "yes"; then
    CFLAGS="$CFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c flags])
  fi
  if test "$GXX" = "yes"; then
    CXXFLAGS="$CXXFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c++ flags])
  fi
fi

define(abtractiface_reqd, [abstract-robot-dynamics >= 1.14.0])
AC_SUBST(ABTRACTIFACE_REQD, "abtractiface_reqd")
PKG_CHECK_MODULES(ABSTRACTROBOTDYNAMICS, abtractiface_reqd)
ABSTRACTROBOTDYNAMICS_DOCDIR=`$PKG_CONFIG abstract-robot-dynamics --variable=docdir`
AC_SUBST(ABSTRACTROBOTDYNAMICS_DOCDIR)

PKG_CHECK_MODULES(KINEOCONTROLLER, KineoController)
KINEOCONTROLLER_PREFIX=`$PKG_CONFIG KineoController --variable=prefix`
AC_SUBST(KINEOCONTROLLER_CFLAGS)
AC_SUBST(KINEOCONTROLLER_LIBS)
AC_SUBST(KINEOCONTROLLER_PREFIX)

AC_SUBST(ROBOTDYNAMICSIMPL_REQUIRE, [])

define(jrldynamics_reqd, [jrl-dynamics >= 1.19])
AC_SUBST([JRL_DYNAMICS_REQD], ["jrldynamics_reqd"])
PKG_CHECK_MODULES([JRL_DYNAMICS], [jrldynamics_reqd])
AC_SUBST([JRL_DYNAMICS_PREFIX])
AC_SUBST([JRL_DYNAMICS_CFLAGS])
AC_SUBST([JRL_DYNAMICS_LIBS])
JRL_DYNAMICS_DOCDIR=`$PKG_CONFIG jrl-dynamics --variable=docdir`
AC_SUBST([JRL_DYNAMICS_DOCDIR])

dnl --- kwsIO ------------------------------------------------

PKG_CHECK_MODULES(KWSIO, kwsIO)
AC_SUBST(KWSIO_CFLAGS)
AC_SUBST(KWSIO_LIBS)

AC_OUTPUT(
	hppModel.pc
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	doc/additionalDoc/package.h
	unitTesting/Makefile
	)
