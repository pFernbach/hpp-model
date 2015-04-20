//
// Copyright (c) 2011, 2012, 2013, 2014 CNRS
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

#ifndef HPP_MODEL_FWD_HH
# define HPP_MODEL_FWD_HH

# include <vector>
# include <list>
# include <map>
# include <Eigen/Core>
# include <hpp/util/pointer.hh>
# include <hpp/fcl/fwd.hh>
# include <hpp/fcl/math/matrix_3f.h>

namespace hpp {
  namespace model {
    HPP_PREDEF_CLASS (Body);
    HPP_PREDEF_CLASS (ChildrenIterator);
    HPP_PREDEF_CLASS (CollisionObject);
    HPP_PREDEF_CLASS (Device);
    HPP_PREDEF_CLASS (DistanceResult);
    HPP_PREDEF_CLASS (HumanoidRobot);
    HPP_PREDEF_CLASS (Joint);
    HPP_PREDEF_CLASS (JointAnchor);
    HPP_PREDEF_CLASS (JointRotation);
    HPP_PREDEF_CLASS (JointSO3);
    HPP_PREDEF_CLASS (JointStaticRod);
    HPP_PREDEF_CLASS (JointConfiguration);
    HPP_PREDEF_CLASS (ObjectFactory);
    HPP_PREDEF_CLASS (ObjectIterator);
    HPP_PREDEF_CLASS (Gripper);
    HPP_PREDEF_CLASS (CenterOfMassComputation);
    enum Request_t {COLLISION, DISTANCE};

    typedef double value_type;
    typedef Eigen::Matrix <value_type, Eigen::Dynamic, 1> vector_t;
    typedef vector_t Configuration_t;
    typedef Eigen::Ref <const Configuration_t> ConfigurationIn_t;
    typedef Eigen::Ref <Configuration_t> ConfigurationOut_t;
    typedef boost::shared_ptr <Configuration_t> ConfigurationPtr_t;
    typedef Eigen::Ref <const vector_t> vectorIn_t;
    typedef Eigen::Ref <vector_t> vectorOut_t;
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> matrix_t;
    typedef Eigen::Ref <matrix_t> matrixOut_t;
    typedef matrix_t::Index size_type;
    typedef fcl::Matrix3f matrix3_t;
    typedef fcl::Vec3f vector3_t;
    typedef Eigen::Matrix <value_type, 6, Eigen::Dynamic> JointJacobian_t;
    typedef Eigen::Matrix <value_type, 3, Eigen::Dynamic> ComJacobian_t;
    typedef Eigen::Block <JointJacobian_t, 3, Eigen::Dynamic>
    HalfJointJacobian_t;

    typedef Body* BodyPtr_t;
    typedef std::vector<Body*> BodyVector_t;
    typedef boost::shared_ptr <CollisionObject> CollisionObjectPtr_t;
    typedef std::list <CollisionObjectPtr_t> ObjectVector_t;
    typedef boost::shared_ptr <Device> DevicePtr_t;
    typedef boost::shared_ptr <const Device> DeviceConstPtr_t;
    typedef std::vector <DistanceResult> DistanceResults_t;
    typedef boost::shared_ptr <HumanoidRobot> HumanoidRobotPtr_t;
    typedef Joint* JointPtr_t;
    typedef JointAnchor* JointAnchorPtr_t;
    typedef JointRotation* JointRotationPtr_t;
    typedef JointSO3* JointSO3Ptr_t;
    template <size_type dimension = 1> class  JointTranslation;
    typedef JointTranslation <1>* JointTranslationPtr_t;
    typedef JointTranslation <2>* JointTranslation2Ptr_t;
    typedef JointTranslation <3>* JointTranslation3Ptr_t;
    typedef const Joint* JointConstPtr_t;
    typedef const JointAnchor* JointAnchorConstPtr_t;
    typedef const JointRotation* JointRotationConstPtr_t;
    typedef const JointSO3* JointSO3ConstPtr_t;
    typedef const JointTranslation <1>* JointTranslationConstPtr_t;
    typedef const JointTranslation <2>* JointTranslation2ConstPtr_t;
    typedef const JointTranslation <3>* JointTranslation3ConstPtr_t;
    typedef std::map <std::string, JointPtr_t> JointByName_t;
    typedef std::vector <JointPtr_t> JointVector_t;
    typedef boost::shared_ptr <Gripper> GripperPtr_t;
    typedef std::vector <GripperPtr_t> Grippers_t;
    typedef fcl::Transform3f Transform3f;
    typedef boost::shared_ptr <CenterOfMassComputation>
      CenterOfMassComputationPtr_t;
  } // namespace model
} // namespace hpp
#endif //HPP_MODEL_FWD_HH
