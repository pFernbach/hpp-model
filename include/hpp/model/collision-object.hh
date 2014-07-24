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
    /// Specialization of fcl::CollisionObject to add a name to objects
    ///
    /// Objects moved by a robot joint. They can collide each other and
    /// distance computation can be computed between them.
    class HPP_MODEL_DLLAPI CollisionObject {
    public:
      /// Create collision object and return shared pointer
      static CollisionObjectPtr_t create (fcl::CollisionObjectPtr_t object,
					  const std::string& name);
      /// Create collision object and return shared pointer
      static CollisionObjectPtr_t create (fcl::CollisionGeometryPtr_t geometry,
					  const Transform3f& position,
					  const std::string& name);
      /// Clone object and attach to given joint.
      CollisionObjectPtr_t clone (const JointPtr_t& joint) const;

      const std::string& name () const {return name_;}
      /// Access to fcl object
      fcl::CollisionObjectPtr_t fcl () const {return object_;}
      /// Access to the joint
      const JointConstPtr_t& joint () const {return joint_;}
      void joint (const JointPtr_t joint);

      /// Return the position in the joint frame
      const fcl::Transform3f& positionInJointFrame () const
      {
	return positionInJointFrame_;
      }

      /// Return transform of the fcl object
      /// \warning If joint linked object -as a robot body- and the robot is
      /// manually moved, this will return the non-update transform.
      /// \note If object is not attached to a joint, use move() to update
      /// transform between hpp and fcl.
      const fcl::Transform3f& getTransform () const
      {
	return object_->getTransform();
      }

      /// Move object to given position
      /// \note If object is attached to a joint, throw exception.
      void move (const Transform3f& position);

    protected:

      /// \name Construction, destruction and copy
      /// \{

      /// Wrap fcl collision object at identity position
      explicit CollisionObject (fcl::CollisionObjectPtr_t object,
				const std::string& name) :
	object_ (object), joint_ (0), name_ (name), weakPtr_ ()
	{
	  positionInJointFrame_.setIdentity ();
	}
      /// Wrap fcl collision object and put at given position
      explicit CollisionObject (fcl::CollisionGeometryPtr_t geometry,
				const Transform3f& position,
				const std::string& name) :
	object_ (new fcl::CollisionObject (geometry, position)),
	joint_ (0), name_ (name), weakPtr_ ()
	{
	  positionInJointFrame_.setIdentity ();
	}

      /// Copy constructor
      explicit CollisionObject (const CollisionObject& object) :
	object_ (object.object_),
	positionInJointFrame_ (object.positionInJointFrame_),
	joint_ (0x0),
	name_ (object.name_),
	weakPtr_ ()
	  {
	  }

      /// \}
      void init (const CollisionObjectWkPtr_t& self)
      {
	weakPtr_ = self;
      }

    private:
      fcl::CollisionObjectPtr_t object_;
      fcl::Transform3f positionInJointFrame_;
      JointConstPtr_t joint_;
      std::string name_;
      CollisionObjectWkPtr_t weakPtr_;
    }; // class CollisionObject
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_COLLISION_OBJECT_HH
