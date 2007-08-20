dnl 
dnl configure.ac file for package hppModel 
dnl Copyright 2007 (c) LAAS-CNRS
dnl 
dnl Author: Florent Lamiraux
dnl

AC_INIT([hppModel], [1.0], [support@company.com])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

PKG_CHECK_MODULES(ABSTRACTROBOTDYNAMICS, abstractRobotDynamics)
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

if test x${with_robotDynamics} = xno; then
	AC_ERROR([please choose an implementation of abstract interfaces for robot dynamic model])
fi

PKG_CHECK_MODULES(ROBOTDYNAMICSIMPL, $with_robotDynamics)
AC_SUBST(ROBOTDYNAMICSIMPL, $with_robotDynamics)
AC_SUBST(ROBOTDYNAMICSIMPL_CFLAGS)
AC_SUBST(ROBOTDYNAMICSIMPL_LIBS)
ROBOTDYNAMICSIMPL_PREFIX=`$PKG_CONFIG $with_robotDynamics --variable=prefix`
AC_SUBST(ROBOTDYNAMICSIMPL_PREFIX)

dnl --- Get name of classes ---------------------------------

CIMPLDYNAMICROBOT=`$PKG_CONFIG $with_robotDynamics --variable=CimplDynamicRobot`
AC_SUBST(CIMPLDYNAMICROBOT)

CIMPLHUMANOIDDYNAMICROBOT=`$PKG_CONFIG $with_robotDynamics --variable=CimplHumanoidDynamicRobot`
AC_SUBST(CIMPLHUMANOIDDYNAMICROBOT)

CIMPLJOINTFREEFLYER=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointFreeFlyer`
AC_SUBST(CIMPLJOINTFREEFLYER)

CIMPLJOINTROTATION=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointRotation`
AC_SUBST(CIMPLJOINTROTATION)

CIMPLJOINTTRANSLATION=`$PKG_CONFIG $with_robotDynamics --variable=CimplJointTranslation`
AC_SUBST(CIMPLJOINTTRANSLATION)



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
