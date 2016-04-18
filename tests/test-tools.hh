// Copyright (C) 2014 LAAS-CNRS
// Author: Steve Tonneau
//
// This file is part of the hpp-rbprm.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#include <sstream>

#define BOOST_TEST_MODULE TEST_CONFIGURATION
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
using boost::test_tools::output_test_stream;

#include <Eigen/Geometry>

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/fcl-to-eigen.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

using hpp::model::Body;
using hpp::model::BodyPtr_t;
using hpp::model::vector_t;
using hpp::model::vectorIn_t;
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
using hpp::model::CollisionObject;
using hpp::model::CollisionObjectPtr_t;
using fcl::CollisionGeometry;
using fcl::CollisionGeometryPtr_t;
typedef Eigen::AngleAxis <value_type> AngleAxis_t;
typedef Eigen::Quaternion <value_type> Quaternion_t;

namespace
{

CollisionObjectPtr_t createCollisionObject(const std::string& name = "obj1")
{
    CollisionGeometryPtr_t root (new fcl::Box (1, 1, 1));
    CollisionObjectPtr_t obstacle = CollisionObject::create
        (root, fcl::Transform3f (), name);
    return obstacle;
}

// Create a robot with various types of joints
DevicePtr_t createRobot ()
{
  DevicePtr_t robot = Device::create ("robot");
  Transform3f position; position.setIdentity ();
  ObjectFactory factory;

  // planar root joint
  JointPtr_t root = factory.createJointTranslation2 (position);
  root->name("rootJoint");
  robot->rootJoint (root);
  root->isBounded (0, true);
  root->lowerBound (0, -1);
  root->upperBound (0, 1);
  root->isBounded (1, true);
  root->lowerBound (1, -1);
  root->upperBound (1, 1);
  BodyPtr_t body = new Body;
  body->name ("root");
  root->setLinkedBody (body);
  body->addInnerObject(createCollisionObject(), true, true);
  // Rotation around z
  position.setQuatRotation (Quaternion3f (sqrt (2)/2, 0, -sqrt (2)/2, 0));
  JointPtr_t joint = factory.createUnBoundedJointRotation (position);
  joint->name("j0");
  root->addChildJoint (joint);
  position.setIdentity ();
  // SO3 joint
  position.setIdentity ();
  JointPtr_t j1 = factory.createJointSO3 (position);
  j1->name ("so3");
  joint->addChildJoint (j1);
  return robot;
}

DevicePtr_t createRobotWithExtraDof ()
{
  DevicePtr_t robot =createRobot();
  robot->setDimensionExtraConfigSpace(3);
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

vector_t slerp (vectorIn_t v0, vectorIn_t v1, const value_type t) {
  Quaternion_t q0(v0[0], v0[1], v0[2], v0[3]);
  Quaternion_t q1(v1[0], v1[1], v1[2], v1[3]);
  Quaternion_t q = q0.slerp (t, q1);
  vector_t res(4);
  res[0] = q.w(); res[1] = q.x(); res[2] = q.y(); res[3] = q.z();
  return res;
}

AngleAxis_t aa (vectorIn_t v0, vectorIn_t v1) {
  Quaternion_t q0(v0[0], v0[1], v0[2], v0[3]);
  Quaternion_t q1(v1[0], v1[1], v1[2], v1[3]);
  return AngleAxis_t (q0 * q1.conjugate ());
}

vector_t interpolation (DevicePtr_t robot, vectorIn_t q0, vectorIn_t q1, int n)
{
  const size_type rso3 = 4;
  bool print = false;

  vector_t q2 (q0);
  value_type u = 0;
  value_type step = (value_type)1 / (value_type)(n + 2);
  vector_t angle1 (n+2), angle2(n+2);

  if (print) std::cout << "HPP  : ";
  for (int i = 0; i < n + 2; ++i) {
    u = 0 + i * step;
    hpp::model::interpolate (robot, q0, q1, u, q2);
    AngleAxis_t aa1 (aa (q0.segment<4>(rso3),q2.segment<4>(rso3)));
    angle1[i] = aa1.angle ();
    if (print) std::cout << aa1.angle () << ", ";
  }
  if (print) std::cout << "\n";
  if (print) std::cout << "Eigen: ";
  for (int i = 0; i < n + 2; ++i) {
    u = 0 + i * step;
    vector_t eigen_slerp = slerp (q0.segment<4>(rso3), q1.segment<4>(rso3), u);
    AngleAxis_t aa2 (aa (q0.segment<4>(rso3),eigen_slerp));
    angle2[i] = aa2.angle ();
    if (print) std::cout << aa2.angle () << ", ";
  }
  if (print) std::cout << "\n";
  return angle1 - angle2;
}
}
