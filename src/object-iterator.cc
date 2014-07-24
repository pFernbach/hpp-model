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

#include <hpp/model/object-iterator.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>

namespace hpp {
  namespace model {
    ObjectIterator::ObjectIterator (Device& device, Request_t type) :
      device_ (device), type_ (type), joints_ (device.getJointVector ()),
      jointIt_ (joints_.begin ())
    {
      for (;jointIt_ != joints_.end (); jointIt_++) {
	BodyPtr_t body = (*jointIt_)->linkedBody ();
	if ((body != 0) &&
	    (objects_ = body->innerObjects (type_)).size () != 0) {
	  ;
	  objIt_ = objects_.begin ();
	  return;
	}
      }
    }

    const CollisionObjectPtr_t& ObjectIterator::operator* ()
    {
      return *objIt_;
    }

    void ObjectIterator::operator++ ()
    {
      objIt_++;
      bool endOfObjects = (objIt_ == objects_.end ());
      while (endOfObjects && jointIt_ != joints_.end ()) {
	BodyPtr_t body = 0x0;
	while (body == 0x0 && jointIt_ != joints_.end ()) {
	  jointIt_++;
	  endOfObjects = false;
	  if (jointIt_ != joints_.end ()) {
	    body = (*jointIt_)->linkedBody ();
	    if (body != 0x0) {
	      objects_ = body->innerObjects (type_);
	      objIt_ = objects_.begin ();
	      endOfObjects = (objIt_ == objects_.end ());
	    }
	  }
	}
      }
    }

    bool ObjectIterator::operator== (const ObjectIterator& other) const
    {
      if (other.jointIt_ == joints_.end () && jointIt_ == joints_.end ())
	return true;
      if (joints_ != other.joints_ || jointIt_ != other.jointIt_) return false;
      if (objects_ != other.objects_ || objIt_ != other.objIt_) return false;
      return true;
    }
    bool ObjectIterator::operator!= (const ObjectIterator& other) const
    {
      return !(*this == other);
    }
    void ObjectIterator::setToEnd ()
    {
      jointIt_ = joints_.end ();
    }
    bool ObjectIterator::isEnd () const
    {
      return jointIt_ == joints_.end ();
    }

  } // namespace model
} // namespace hpp
