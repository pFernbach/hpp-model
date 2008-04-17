dnl 
dnl configure.ac file for package hppModel 
dnl Copyright 2007 (c) LAAS-CNRS
dnl 
dnl Author: Florent Lamiraux
dnl

AC_INIT([hppModel], [1.0.99], [hpp@laas.fr])
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

define(abtractiface_reqd, [abstractRobotDynamics >= 1.8])
AC_SUBST(ABTRACTIFACE_REQD, "abtractiface_reqd")
PKG_CHECK_MODULES(ABSTRACTROBOTDYNAMICS, abtractiface_reqd)
ABSTRACTROBOTDYNAMICS_PREFIX=`$PKG_CONFIG abstractRobotDynamics --variable=prefix`
AC_SUBST(ABSTRACTROBOTDYNAMICS_PREFIX)

PKG_CHECK_MODULES(KINEOCONTROLLER, KineoController)
KINEOCONTROLLER_PREFIX=`$PKG_CONFIG KineoController --variable=prefix`
AC_SUBST(KINEOCONTROLLER_CFLAGS)
AC_SUBST(KINEOCONTROLLER_LIBS)
AC_SUBST(KINEOCONTROLLER_PREFIX)

AC_SUBST(ROBOTDYNAMICSIMPL_REQUIRE, [])

dnl --- Implementation of jrlAbstractRobotDynamics --------------------

AC_ARG_WITH(robotDynamics, AC_HELP_STRING([--with-robotDynamics], 
	[which implementation of abstractRobotDynamics to use]),,with_robotDynamics=no)

case "${with_robotDynamics}" in
   dynamicsJRLJapan) ;;
   yes|no|*)
	AC_MSG_WARN([You must choose an implementation of abstract interfaces])
	AC_MSG_WARN([for robot dynamic model. Available implementations are:])
	AC_MSG_WARN([	dynamicsJRLJapan])
	AC_MSG_ERROR([--with-robotDynamics option missing])
	;;
esac

PKG_CHECK_MODULES(ROBOTDYNAMICSIMPL, $with_robotDynamics)
AC_SUBST(ROBOTDYNAMICSIMPL, $with_robotDynamics)
AC_SUBST(ROBOTDYNAMICSIMPL_CFLAGS)
AC_SUBST(ROBOTDYNAMICSIMPL_LIBS)
ROBOTDYNAMICSIMPL_PREFIX=`$PKG_CONFIG $with_robotDynamics --variable=prefix`
AC_SUBST(ROBOTDYNAMICSIMPL_PREFIX)

dnl --- Get name of classes ---------------------------------

dnl CIMPLDYNAMICROBOT=`$PKG_CONFIG $with_robotDynamics --variable=CimplDynamicRobot`
dnl AC_SUBST(CIMPLDYNAMICROBOT)

dnl CIMPLHUMANOIDDYNAMICROBOT=`$PKG_CONFIG $with_robotDynamics --variable=CimplHumanoidDynamicRobot`
dnl AC_SUBST(CIMPLHUMANOIDDYNAMICROBOT)

dnl CIMPLJOINTFREEFLYER=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointFreeFlyer`
dnl AC_SUBST(CIMPLJOINTFREEFLYER)

dnl CIMPLJOINTROTATION=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointRotation`
dnl AC_SUBST(CIMPLJOINTROTATION)

dnl CIMPLJOINTTRANSLATION=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointTranslation`
dnl AC_SUBST(CIMPLJOINTTRANSLATION)

dnl --- kwsIO ------------------------------------------------

PKG_CHECK_MODULES(KWSIO, kwsIO)
AC_SUBST(KWSIO_CFLAGS)
AC_SUBST(KWSIO_LIBS)

AC_OUTPUT(
	hppModel.pc
	Makefile
	include/Makefile
	include/hppModel/hppImplRobotDynamics.h
	src/Makefile
	doc/Makefile
	doc/package.dox
	doc/additionalDoc/package.h
	unitTesting/Makefile
	)
