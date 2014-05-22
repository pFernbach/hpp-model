#
# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux
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

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  body.cc
  collision-object.cc
  device.cc
  humanoid-robot.cc
  joint.cc
  joint-configuration.cc
  object-iterator.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} fcl)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} jrl-mathtools)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} eigen3)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
