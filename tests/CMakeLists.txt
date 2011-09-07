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

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
# HPP_MODEL_TEST(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(HPP_MODEL_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-dynamics)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} kwsIO)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-util)

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
  ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.kab
    DEPEND ${NAME}
    COMMAND KineoAddonBuilder ARGS -a ${CMAKE_CURRENT_BINARY_DIR}/${NAME})
  ADD_CUSTOM_TARGET(license DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.kab)
  ADD_DEPENDENCIES(license ${CMAKE_CURRENT_BINARY_DIR}/${NAME})
ENDMACRO(HPP_MODEL_TEST)

# tests.
HPP_MODEL_TEST(load-nao)
