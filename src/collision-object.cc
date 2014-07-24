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

#include <hpp/util/debug.hh>
#include <hpp/model/fwd.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace model {

    CollisionObjectPtr_t
    CollisionObject::create (fcl::CollisionObjectPtr_t object,
			     const std::string& name)
    {
      CollisionObject* ptr = new CollisionObject (object, name);
      CollisionObjectPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    CollisionObjectPtr_t
    CollisionObject::create (fcl::CollisionGeometryPtr_t geometry,
			     const Transform3f& position,
			     const std::string& name)
    {
      CollisionObject* ptr = new CollisionObject (geometry, position, name);
      CollisionObjectPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }


    CollisionObjectPtr_t CollisionObject::clone (const JointPtr_t& joint) const
    {
      CollisionObject* ptr = new CollisionObject (*this);
      CollisionObjectPtr_t shPtr (ptr);
      ptr->init (shPtr);
      ptr->joint_ = joint;
      return shPtr;
    }

    // -----------------------------------------------------------------------

    void CollisionObject::joint (const JointPtr_t joint)
    {
      joint_ = joint;
      if (joint_) {
	fcl::Transform3f jointPosition = joint->currentTransformation ();
	positionInJointFrame_ = jointPosition.inverse () *
	  object_->getTransform ();
      } else {
	positionInJointFrame_ = object_->getTransform ();
      }
    }

    // -----------------------------------------------------------------------

    void CollisionObject::move (const Transform3f& position)
    {
      if (joint_) {
	throw std::runtime_error ("Cannot move object attached to a joint");
      }
      positionInJointFrame_ = position;
      hppDout (info, "transform = " << position);
      object_->setTransform (positionInJointFrame_);
    }

  } // namespace model
} // namespace hpp
