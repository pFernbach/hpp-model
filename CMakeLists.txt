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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-model)
SET(PROJECT_DESCRIPTION
  "Implementation of hybrid geometric and dynamic humanoid robots"
  )
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

# Declare headers
SET(${PROJECT_NAME}_HEADERS
  include/hpp/model/body.hh
  include/hpp/model/children-iterator.hh
  include/hpp/model/collision-object.hh
  include/hpp/model/configuration.hh
  include/hpp/model/device.hh
  include/hpp/model/distance-result.hh
  include/hpp/model/fcl-to-eigen.hh
  include/hpp/model/fwd.hh
  include/hpp/model/humanoid-robot.hh
  include/hpp/model/joint.hh
  include/hpp/model/joint-configuration.hh
  include/hpp/model/object-factory.hh
  include/hpp/model/object-iterator.hh
  )

# Declare dependencies
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2")
ADD_REQUIRED_DEPENDENCY("jrl-mathtools >= 1.5")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 0.7")
ADD_REQUIRED_DEPENDENCY("fcl >= 0.2.9")

# Compile documentation
CONFIGURE_FILE(doc/main.hh.in doc/main.hh)

# Add dependency toward hpp-model library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-model")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
