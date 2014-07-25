//
// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
//
//
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MODEL_CONFIGURATION_HH
# define HPP_MODEL_CONFIGURATION_HH

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>

namespace hpp {
  namespace model {
    /// Integrate a constant velocity during unit time.
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param configuration initial and result configurations
    /// \param velocity velocity vector
    /// \retval result configuration reached after integration.
    /// Velocity is dispatched to each joint that integrates according to its
    /// Lie group structure, i.e.
    /// \li \f$q_i += v_i\f$ for translation joint and bounded rotation joints,
    /// \li \f$q_i += v_i \mbox{ modulo } 2\pi\f$ for unbounded rotation joints,
    /// \li constant rotation velocity for SO(3) joints.
    ///
    /// \note bounded degrees of freedom are saturated if the result of the
    ///       above operation is beyond a bound.
    inline void integrate  (const DevicePtr_t& robot,
			    ConfigurationIn_t configuration,
			    vectorIn_t velocity, ConfigurationOut_t result)
    {
      const JointVector_t& jv (robot->getJointVector ());
      for (model::JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); itJoint++) {
	size_type indexConfig = (*itJoint)->rankInConfiguration ();
	size_type indexVelocity = (*itJoint)->rankInVelocity ();
	(*itJoint)->configuration ()->integrate (configuration, velocity,
						 indexConfig, indexVelocity,
						 result);
      }
    }

    /// Difference between two configurations as a vector
    ///
    /// \param robot robot that describes the kinematic chain
    /// \param q1 first configuration,
    /// \param q2 second configuration,
    /// \retval result difference as a vector \f$\textbf{v}\f$ such that
    /// q2 is the result of method integrate from q1 with vector
    /// $\f\textbf{v}\f$
    void inline difference (const DevicePtr_t& robot, ConfigurationIn_t q1,
			    ConfigurationIn_t q2, vectorOut_t result)
    {
      const JointVector_t& jv (robot->getJointVector ());
      for (model::JointVector_t::const_iterator itJoint = jv.begin ();
	   itJoint != jv.end (); itJoint++) {
	size_type indexConfig = (*itJoint)->rankInConfiguration ();
	size_type indexVelocity = (*itJoint)->rankInVelocity ();
	(*itJoint)->configuration ()->difference (q1, q2, indexConfig,
						  indexVelocity, result);
      }
    }
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_CONFIGURATION_HH
