#
# Copyright (c) 2011 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-model
# hpp-model is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-model is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-model  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

EXEC_PROGRAM("${PKG_CONFIG_EXECUTABLE}" ARGS
  "--variable=moduledir" KineoController
  OUTPUT_VARIABLE "KINEO_MODULE_DIR")
EXEC_PROGRAM("${PKG_CONFIG_EXECUTABLE}" ARGS
  "--variable=debug_suffix" KineoController
  OUTPUT_VARIABLE "KINEO_DEBUG_SUFFIX")
STRING(REGEX REPLACE "\n" "" KINEO_DEBUG_SUFFIX "${KINEO_DEBUG_SUFFIX}")
STRING(REGEX REPLACE "\n" "" KINEO_MODULE_DIR "${KINEO_MODULE_DIR}")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

CONFIG_FILES(module-dir.hh)

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  anchor-joint.cc
  body.cc
  body-factory.cc
  capsule-body.cc
  capsule-body-factory.cc
  device.cc
  freeflyer-joint.cc
  humanoid-robot.cc
  joint.cc
  parser.cc
  rotation-joint.cc
  translation-joint.cc
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} kwsIO)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-geometry)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
