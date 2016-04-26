///
/// Copyright (c) 2016 CNRS
/// Author: Steve Tonneau
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
//   - Builds and clones collision-object,
//   - asserts that they can displaced treated independtly
//   - while the core geometry object remains the same
//   - Similarly, it builds a robot
//   - and clones it, and tests that the robot
//   - are equivalent and independent

#include "test-tools.hh"
#include <hpp/fcl/collision.h>


BOOST_AUTO_TEST_CASE(clone_collision_object)
{
  JointPtr_t joint(0);
  CollisionObjectPtr_t c1 = createCollisionObject();
  CollisionObjectPtr_t c2 =  c1->clone(joint);
  BOOST_CHECK_EQUAL(c1->name(), c2->name());
  BOOST_CHECK_EQUAL(c1->fcl()->collisionGeometry().get(), c2->fcl()->collisionGeometry().get());
  BOOST_CHECK_NE(c1->fcl().get(), c2->fcl().get());

  fcl::CollisionResult collisionResult;
  fcl::CollisionRequest collisionRequest;
  fcl::collide(c1->fcl().get(),c2->fcl().get(),collisionRequest,collisionResult);
  BOOST_CHECK_MESSAGE ( collisionResult.isCollision(),
      "Object and clone should be colliding in default position");

  fcl::CollisionResult colRes2;
  fcl::CollisionRequest colReq2;
  fcl::Transform3f transform;
  transform.setTranslation(fcl::Vec3f(20,2,2));
  c1->move(transform);
  fcl::collide(c1->fcl().get(),c2->fcl().get(),colReq2,colRes2);
  BOOST_CHECK_MESSAGE ( !colRes2.isCollision(),
      "Object and clone should not be colliding after translation");
}

BOOST_AUTO_TEST_CASE(clone_device)
{
  DevicePtr_t d1 = createRobotWithExtraDof();
  DevicePtr_t d2 = d1->clone();
  BOOST_CHECK_EQUAL(d1->name(), d2->name());
  BOOST_CHECK_EQUAL(d1->extraConfigSpace().dimension(), d2->extraConfigSpace().dimension());
  BOOST_CHECK_EQUAL(d1->configSize(), d2->configSize());
  JointVector_t jv1 = d1->getJointVector();
  JointVector_t jv2 = d2->getJointVector();
  BOOST_CHECK_EQUAL(jv1.size(),jv2.size());

  JointPtr_t jtree1(d1->rootJoint()), jtree2(d2->rootJoint());
  while(jtree1 && jtree2)
  {
    BOOST_CHECK_EQUAL(jtree1->name(), jtree2->name());
    BOOST_CHECK_EQUAL(jtree1->numberChildJoints(), jtree2->numberChildJoints());
    jtree1 = (jtree1->numberChildJoints() > 0) ? (jtree1->childJoint(0)) : 0;
    jtree2 = (jtree2->numberChildJoints() > 0) ? (jtree2->childJoint(0)) : 0;
  }

  d1->computeForwardKinematics();
  d2->computeForwardKinematics();


  // now verifying that transforms are correctly updated as well
  fcl::Vec3f x(0,0,1);
  fcl::Transform3f r1, r2;

  for(std::size_t i =0; i != jv1.size(); ++i)
  {
      r1 = jv1[i]->currentTransformation();
      r2 = jv2[i]->currentTransformation();
      BOOST_CHECK_MESSAGE((r1.getRotation() * x + r1.getTranslation() - r2.getRotation() * x + r2.getTranslation()).norm() < 10e-2,
              "Transformation not matching for joint " +jv1[i]->name()+ " and " +jv2[i]->name() );
  }

  //collision checking with objects
  fcl::CollisionObjectPtr_t c1 = d1->rootJoint()->linkedBody()->innerObjects(hpp::model::COLLISION).front()->fcl();
  fcl::CollisionObjectPtr_t c2 = d2->rootJoint()->linkedBody()->innerObjects(hpp::model::COLLISION).front()->fcl();

  fcl::CollisionResult collisionResult;
  fcl::CollisionRequest collisionRequest;
  fcl::collide(c1.get(),c2.get(),collisionRequest,collisionResult);
  BOOST_CHECK_MESSAGE ( collisionResult.isCollision(),
      "Object and clone should be colliding in default position");

  Configuration_t config = d2->currentConfiguration();
  config.head(2)= Eigen::Vector2d(5,5);
  d2->currentConfiguration(config); d2->computeForwardKinematics();

  fcl::CollisionResult colRes2;
  fcl::CollisionRequest colReq2;
  fcl::collide(c1.get(),c2.get(),colReq2,colRes2);
  BOOST_CHECK_MESSAGE ( !colRes2.isCollision(),
      "Device robot and clone should not be colliding after collision");

}
