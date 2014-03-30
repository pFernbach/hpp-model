///
/// Copyright (c) 2013, 2014 CNRS
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

#include <hpp/model/fwd.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace model {
    CollisionObject::CollisionObject (fcl::CollisionGeometryPtr_t geometry,
				      const Transform3f& position,
				      const std::string& name) :
      object_ (new fcl::CollisionObject (geometry, position)),
      joint_ (0), name_ (name)
    {
      positionInJointFrame_.setIdentity ();
    }

    // -----------------------------------------------------------------------

    void CollisionObject::joint (const JointPtr_t joint)
    {
      assert (joint);
      joint_ = joint;
      fcl::Transform3f jointPosition = joint->currentTransformation ();
      positionInJointFrame_ = jointPosition.inverse () *
	object_->getTransform ();
    }

    void CollisionObject::move (const Transform3f& position)
    {
      if (joint_) {
	throw std::runtime_error ("Cannot move object attached to a joint");
      }
      positionInJointFrame_ = position;
      object_->setTransform (positionInJointFrame_);
    }

  } // namespace model
} // namespace hpp
