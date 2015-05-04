///
/// Copyright (c) 2011 CNRS
/// Author: Florent Lamiraux
///
///
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

// This test
//   - builds a robot with various types of joints,
//   - randomly samples pairs of configurations,
//   - test consistency between hpp::model::difference and
//     hpp::model::integrate functions.

#include <sstream>

#define BOOST_TEST_MODULE TEST_CONFIGURATION
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
using boost::test_tools::output_test_stream;

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/object-factory.hh>

using hpp::model::vector_t;
using hpp::model::Configuration_t;
using hpp::model::JointPtr_t;
using hpp::model::ObjectFactory;
using hpp::model::Transform3f;
using fcl::Quaternion3f;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointVector_t;
using hpp::model::ConfigurationPtr_t;
using hpp::model::size_type;
using hpp::model::value_type;

// Create a robot with various types of joints
DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("robot");
  Transform3f position; position.setIdentity ();
  ObjectFactory factory;

  // planar root joint
  JointPtr_t root = factory.createJointTranslation2 (position);
  robot->rootJoint (root);
  root->isBounded (0, true);
  root->lowerBound (0, -1);
  root->upperBound (0, 1);
  root->isBounded (1, true);
  root->lowerBound (1, -1);
  root->upperBound (1, 1);
  // Rotation around z
  position.setQuatRotation (Quaternion3f (sqrt (2)/2, 0, -sqrt (2)/2, 0));
  JointPtr_t joint = factory.createUnBoundedJointRotation (position);
  root->addChildJoint (joint);
  position.setIdentity ();
  // SO3 joint
  position.setIdentity ();
  JointPtr_t j1 = factory.createJointSO3 (position);
  joint->addChildJoint (j1);

  return robot;
}

void shootRandomConfig (const DevicePtr_t& robot, Configuration_t& config)
{
  JointVector_t jv = robot->getJointVector ();
  for (JointVector_t::const_iterator itJoint = jv.begin ();
       itJoint != jv.end (); itJoint++) {
    std::size_t rank = (*itJoint)->rankInConfiguration ();
    (*itJoint)->configuration ()->uniformlySample (rank, config);
  }
  // Shoot extra configuration variables
  size_type extraDim = robot->extraConfigSpace ().dimension ();
  size_type offset = robot->configSize () - extraDim;
  for (size_type i=0; i<extraDim; ++i) {
    value_type lower = robot->extraConfigSpace ().lower (i);
    value_type upper = robot->extraConfigSpace ().upper (i);
    value_type range = upper - lower;
    if ((range < 0) ||
	(range == std::numeric_limits<double>::infinity())) {
      std::ostringstream oss
	("Cannot uniformy sample extra config variable ");
      oss << i << ". min = " << ", max = " << upper << std::endl;
      throw std::runtime_error (oss.str ());
    }
    config [offset + i] = (upper - lower) * rand ()/RAND_MAX;
  }
}

BOOST_AUTO_TEST_CASE(difference_and_integrate)
{
  DevicePtr_t robot = createRobot ();
  Configuration_t q0; q0.resize (robot->configSize ());
  Configuration_t q1; q1.resize (robot->configSize ());
  Configuration_t q2; q2.resize (robot->configSize ());
  vector_t q1_minus_q0; q1_minus_q0.resize (robot->numberDof ());
  for (size_type i=0; i<10000; ++i) {
    shootRandomConfig (robot, q0);
    shootRandomConfig (robot, q1);
    hpp::model::difference (robot, q1, q0, q1_minus_q0);
    hpp::model::integrate (robot, q0, q1_minus_q0, q2);
    std::cout << "q1=" << q1.transpose () << std::endl;
    std::cout << "q2=" << q2.transpose () << std::endl;
    std::cout << "(q2 - q1).norm () = " << (q2 - q1).norm () << std::endl;
    BOOST_CHECK ((q2 - q1).norm () < 1e-10);
  }
}
