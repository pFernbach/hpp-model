//
// Copyright (c) 2013, 2014 CNRS
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

#ifndef HPP_MODEL_OBJECT_FACTORY_HH
# define HPP_MODEL_OBJECT_FACTORY_HH

# include <hpp/model/config.hh>
# include <hpp/model/body.hh>
# include <hpp/model/humanoid-robot.hh>
# include <hpp/model/joint.hh>

namespace hpp {
  namespace model {
    /// Object Factory
    ///
    /// Create instances of objects.
    class HPP_MODEL_DLLAPI ObjectFactory
    {
    public:
      ObjectFactory ()
      {}
      virtual ~ObjectFactory () {}
 
      virtual DevicePtr_t createRobot (const std::string& name)
      {
	return Device::create (name);
      }
      virtual HumanoidRobotPtr_t createHumanoidRobot (const std::string& name)
      {
	return HumanoidRobot::create (name);
      }
      virtual JointPtr_t createJointSO3 (const Transform3f& initialPosition)
      {
	return new JointSO3 (initialPosition);
      }
      virtual JointPtr_t createJointAnchor (const Transform3f& initialPosition)
      {
	return new JointAnchor (initialPosition);
      }
      virtual JointPtr_t createJointRotation
	(const Transform3f& initialPosition)
      {
	return new JointRotation (initialPosition);
      }
      virtual JointPtr_t createJointTranslation
      (const Transform3f& initialPosition)
      {
	return new JointTranslation (initialPosition);
      }
      virtual BodyPtr_t createBody ()
      {
	return new Body;
      }
    }; // class ObjectFactory
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_OBJECT_FACTORY_HH
