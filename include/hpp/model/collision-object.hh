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

#ifndef HPP_MODEL_COLLISION_OBJECT_HH
# define HPP_MODEL_COLLISION_OBJECT_HH

# include <fcl/collision_object.h>
# include <hpp/util/pointer.hh>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>

namespace fcl {
  HPP_PREDEF_CLASS (CollisionObject);
  HPP_PREDEF_CLASS (CollisionGeometry);
} // namespace fcl

namespace hpp {
  namespace model {
    /// Specialization if fcl::CollisionObject to add a name to objects
    class HPP_MODEL_DLLAPI CollisionObject {
    public:
      explicit CollisionObject (fcl::CollisionObjectShPtr object,
				const std::string& name) :
	object_ (object), joint_ (0), name_ (name)
      {
	positionInJointFrame_.setIdentity ();
      }
      explicit CollisionObject (fcl::CollisionGeometryShPtr geometry,
				const Transform3f& position,
				const std::string& name);
      const std::string& name () const {return name_;}
      /// Access to fcl object
      fcl::CollisionObjectShPtr fcl () const {return object_;}
      const Joint* joint () const {return joint_;}
      void joint (const Joint* joint);
      const fcl::Transform3f& positionInJointFrame () const
      {
	return positionInJointFrame_;
      }
      /// Move object to given position
      /// \note If object is attached to a joint, throw exception.
      void move (const Transform3f& position);
    private:
      fcl::CollisionObjectShPtr object_;
      fcl::Transform3f positionInJointFrame_;
      const Joint* joint_;
      std::string name_;
    }; // class CollisionObject
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_COLLISION_OBJECT_HH
