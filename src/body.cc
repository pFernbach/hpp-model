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

#include <fcl/distance.h>
#include <fcl/collision.h>
#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/object-factory.hh>

namespace fcl {
  HPP_PREDEF_CLASS (CollisionGeometry);
} // namespace fcl

namespace hpp {
  namespace model {

    //-----------------------------------------------------------------------

    static ObjectVector_t::iterator
    findObject (ObjectVector_t& vector,
		const fcl::CollisionObjectConstPtr_t& object)
    {
      ObjectVector_t::iterator it;
      for (it = vector.begin (); it != vector.end (); it++) {
	const CollisionObjectPtr_t& local = *it;
	if (local->fcl ()->collisionGeometry () ==
	    object->collisionGeometry ()) return it;
      }
      return it;
    }

    //-----------------------------------------------------------------------

    Body:: Body () : collisionInnerObjects_ (), collisionOuterObjects_ (),
		     distanceInnerObjects_ (), distanceOuterObjects_ (),
		     joint_ (0x0), name_ (), localCom_ (), inertiaMatrix_ (),
		     mass_ (0), radius_ (0)
    {
    }

    //-----------------------------------------------------------------------

    Body::Body (const Body& body) :
      collisionInnerObjects_ (), collisionOuterObjects_ (),
      distanceInnerObjects_ (), distanceOuterObjects_ (),
      joint_ (0x0), name_ (body.name_), localCom_ (body.localCom_),
      inertiaMatrix_ (body.inertiaMatrix_), mass_ (body.mass_),
      radius_ (body.radius_)
    {
    }

    //-----------------------------------------------------------------------

    BodyPtr_t Body::clone (const JointPtr_t& joint) const
    {
      BodyPtr_t newBody = new Body (*this);
      joint->setLinkedBody (newBody);
      // Copy collision object lists
      for (ObjectVector_t::const_iterator itObj =
	     collisionInnerObjects_.begin ();
	   itObj != collisionInnerObjects_.end (); ++itObj) {
	newBody->collisionInnerObjects_.push_back ((*itObj)->clone (joint));
      }
      for (ObjectVector_t::const_iterator itObj =
	     collisionOuterObjects_.begin ();
	   itObj != collisionOuterObjects_.end (); ++itObj) {
	newBody->collisionOuterObjects_.push_back ((*itObj)->clone (joint));
      }
      for (ObjectVector_t::const_iterator itObj =
	     distanceInnerObjects_.begin ();
	   itObj != distanceInnerObjects_.end (); ++itObj) {
	newBody->distanceInnerObjects_.push_back ((*itObj)->clone (joint));
      }
      for (ObjectVector_t::const_iterator itObj =
	     distanceOuterObjects_.begin ();
	   itObj != distanceOuterObjects_.end (); ++itObj) {
	newBody->distanceOuterObjects_.push_back ((*itObj)->clone (joint));
      }
      return newBody;
    }

    //-----------------------------------------------------------------------

    void Body::updateRadius (const CollisionObjectPtr_t& object)
    {
      fcl::CollisionGeometryConstPtr_t geom =
	object->fcl ()->collisionGeometry();
      const Transform3f& positionInJoint = object->positionInJointFrame ();
      fcl::Vec3f p;
      p [0] = geom->aabb_local.min_ [0];
      p [1] = geom->aabb_local.min_ [1];
      p [2] = geom->aabb_local.min_ [2];
      value_type newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.max_ [0];
      p [1] = geom->aabb_local.min_ [1];
      p [2] = geom->aabb_local.min_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.min_ [0];
      p [1] = geom->aabb_local.max_ [1];
      p [2] = geom->aabb_local.min_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.max_ [0];
      p [1] = geom->aabb_local.max_ [1];
      p [2] = geom->aabb_local.min_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.min_ [0];
      p [1] = geom->aabb_local.min_ [1];
      p [2] = geom->aabb_local.max_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.max_ [0];
      p [1] = geom->aabb_local.min_ [1];
      p [2] = geom->aabb_local.max_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.min_ [0];
      p [1] = geom->aabb_local.max_ [1];
      p [2] = geom->aabb_local.max_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;

      p [0] = geom->aabb_local.max_ [0];
      p [1] = geom->aabb_local.max_ [1];
      p [2] = geom->aabb_local.max_ [2];
      newLength = positionInJoint.transform (p).length ();
      if (newLength > radius_) radius_ = newLength;
      hppDout (info, "joint " << joint_->name () << ", radius " << radius_);
    }

    //-----------------------------------------------------------------------

    void Body::addInnerObject (const CollisionObjectPtr_t& object,
			       bool collision, bool distance)
    {
      if (collision) {
	if (findObject (collisionInnerObjects_,	object->fcl ()) ==
	    collisionInnerObjects_.end ()) {
	  if (joint () == 0) {
	    throw std::runtime_error ("Body should be connected to a joint "
				      "before inserting objects.");
	  }
	  object->joint (joint ());
	  updateRadius (object);
	  collisionInnerObjects_.push_back (object);
	}
      }
      if (distance) {
	if (findObject (distanceInnerObjects_, object->fcl ()) ==
	    distanceInnerObjects_.end ()) {
	  if (joint () == 0) {
	    throw std::runtime_error ("Body should be connected to a joint "
				      "before inserting objects.");
	  }
	  object->joint (joint ());
	  distanceInnerObjects_.push_back (object);
	  if (!joint ()->robot ()) {
	    throw std::runtime_error ("Body should be connected to a robot "
				      "before inserting inner objects.");
	  }
	  joint ()->robot ()->updateDistances ();
	}
      }
    }

    //-----------------------------------------------------------------------

    void Body::addOuterObject (const CollisionObjectPtr_t& object,
			       bool collision, bool distance)
    {
      if (collision) {
	if (findObject (collisionOuterObjects_,	object->fcl ()) ==
	    collisionOuterObjects_.end ()) {
	  hppDout (info, "adding " << object->name () << " to body "
		   << this->name_ << " for collision");
	  collisionOuterObjects_.push_back (object);
	}
      }
      if (distance) {
	if (findObject (distanceOuterObjects_, object->fcl ()) ==
	    distanceOuterObjects_.end ()) {
	  hppDout (info, "adding " << object->name () << " to body "
		   << this->name_ << " for distance");
	  distanceOuterObjects_.push_back (object);
	  if (!joint ()->robot ()) {
	    throw std::runtime_error ("Body should be connected to a robot "
				      "before inserting outer objects.");
	  }
	  joint ()->robot ()->updateDistances ();
	}
      }
    }

    //-----------------------------------------------------------------------

    void Body::removeInnerObject (const CollisionObjectPtr_t& object,
				  bool collision, bool distance)
    {
      if (collision) {
	ObjectVector_t::iterator it =
	  findObject (collisionInnerObjects_, object->fcl ());
	if (it != collisionInnerObjects_.end ())
	  collisionInnerObjects_.erase (it);
      }
      if (distance) {
	ObjectVector_t::iterator it =
	  findObject (distanceInnerObjects_, object->fcl ());
	if (it != distanceInnerObjects_.end ())
	  distanceInnerObjects_.erase (it);
	if (!joint ()->robot ()) {
	  throw std::runtime_error ("Body should be connected to a robot "
				    "before inserting outer objects.");
	}
	joint ()->robot ()->updateDistances ();
      }
    }

    //-----------------------------------------------------------------------

    void Body::removeOuterObject (const CollisionObjectPtr_t& object,
				  bool collision, bool distance)
    {
      if (collision) {
	ObjectVector_t::iterator it =
	  findObject (collisionOuterObjects_, object->fcl ());
	if (it != collisionOuterObjects_.end ()) {
	  collisionOuterObjects_.erase (it);
	}
      }
      if (distance) {
	ObjectVector_t::iterator it =
	  findObject (distanceOuterObjects_, object->fcl ());
	if (it != distanceOuterObjects_.end ()) {
	  distanceOuterObjects_.erase (it);
	}
	if (!joint ()->robot ()) {
	  throw std::runtime_error ("Body should be connected to a robot "
				    "before inserting outer objects.");
	}
	joint ()->robot ()->updateDistances ();
      }
    }

    //-----------------------------------------------------------------------

    const ObjectVector_t& Body::innerObjects (Request_t type) const
    {
      switch (type) {
      case COLLISION:
	return collisionInnerObjects_;
      case DISTANCE:
	return distanceInnerObjects_;
      default:
	throw std::runtime_error
	  ("Please choose between COLLISION and DISTANCE.");
      }
    }

    //-----------------------------------------------------------------------

    const ObjectVector_t& Body::outerObjects (Request_t type) const
    {
      switch (type) {
      case COLLISION:
	return collisionOuterObjects_;
      case DISTANCE:
	return distanceOuterObjects_;
      default:
	throw std::runtime_error
	  ("Please choose between COLLISION and DISTANCE.");
      }
    }

    bool Body::collisionTest () const
    {
      fcl::CollisionRequest collisionRequest;
      fcl::CollisionResult collisionResult;
      for (ObjectVector_t::const_iterator itInner =
	     collisionInnerObjects_.begin ();
	   itInner != collisionInnerObjects_.end (); itInner++) {
	for (ObjectVector_t::const_iterator itOuter =
	       collisionOuterObjects_.begin ();
	     itOuter != collisionOuterObjects_.end (); itOuter++) {
	  if (fcl::collide ((*itInner)->fcl ().get (),
			    (*itOuter)->fcl ().get (),
			    collisionRequest, collisionResult) != 0) {
	    return true;
	  }
	}
      }
      return false;
    }

    void Body::computeDistances (DistanceResults_t& results,
				 DistanceResults_t::size_type& offset)
    {
      fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
      for (ObjectVector_t::iterator itInner = distanceInnerObjects_.begin ();
	   itInner != distanceInnerObjects_.end (); itInner++) {
	// Compute global position if inner object
	fcl::Transform3f globalPosition = joint ()->currentTransformation ()*
	  (*itInner)->positionInJointFrame ();
	(*itInner)->fcl ()->setTransform (globalPosition);
	for (ObjectVector_t::iterator itOuter = distanceOuterObjects_.begin ();
	     itOuter != distanceOuterObjects_.end (); itOuter++) {
	  // Compute global position if inner object
	  results [offset].fcl.clear ();
	  fcl::distance ((*itInner)->fcl ().get (), (*itOuter)->fcl ().get (),
			 distanceRequest, results [offset].fcl);
	  results [offset].innerObject = *itInner;
	  results [offset].outerObject = *itOuter;
	  offset++;
	}
      }
    }
  } // namespace model
} // namespace hpp
