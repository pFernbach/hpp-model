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

#include <hpp/model/collision-object.hh>
#include <hpp/model/device.hh>
#include <hpp/model/fcl-to-eigen.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/gripper.hh>

namespace hpp {
  namespace model {

    static Transform3f I4;

    Device::Device(const std::string& name) :
      name_ (name), distances_ (),
      jointByName_ (),
      jointVector_ (), jointByConfigRank_ (), jointByVelocityRank_ (),
      rootJoint_ (0x0), numberDof_ (0),
      configSize_ (0), currentConfiguration_ (configSize_),
      currentVelocity_ (numberDof_), 	currentAcceleration_ (numberDof_),
      com_ (), jacobianCom_ (3, 0), mass_ (0), upToDate_ (false),
      computationFlag_ (ALL), collisionPairs_ (), distancePairs_ (),
      grippers_ (),q0_(configSize_), weakPtr_ ()
    {
      com_.setZero ();
      I4.setIdentity ();
    }
    Device::~Device()
    {
    }

    // ========================================================================

    DevicePtr_t Device::create(std::string name)
    {
      Device* ptr = new Device (name);
      DevicePtr_t shPtr (ptr);

      ptr->init (shPtr);
      return shPtr;
    }

    // ========================================================================

    DevicePtr_t Device::createCopy(const DevicePtr_t& device)
    {
      Device* ptr = new Device(device->name());
      DevicePtr_t shPtr(ptr);

      ptr->initCopy (shPtr, *device);
      return shPtr;
    }

    // ========================================================================

    namespace
    {
    void CloneJointRec(JointPtr_t current, JointPtr_t clone)
    {
        for(std::size_t i = 0; i < current->numberChildJoints(); ++i)
        {
            JointPtr_t cloneChild = current->childJoint(i)->clone();
            clone->addChildJoint(cloneChild);
            CloneJointRec(current->childJoint(i),cloneChild);
        }
    }

    JointPtr_t CloneJoints(hpp::model::Device &clone, const hpp::model::Device &device)
    {
        JointPtr_t root(device.rootJoint()->clone());
        clone.rootJoint(root);
        CloneJointRec(device.rootJoint(), root);
        return root;
    }

    JointVector_t CloneDisabledPositions(const hpp::model::Device &clone, const JointVector_t& disabledPositions)
    {
        JointVector_t res;
        for(JointVector_t::const_iterator cit = disabledPositions.begin();
            cit != disabledPositions.end(); ++cit)
        {
            res.push_back(clone.getJointByName((*cit)->name()));
        }
        return res;
    }

    Grippers_t CloneGrippers(hpp::model::Device &clone, const hpp::model::Device &device)
    {
        Grippers_t res;
        for(Grippers_t::const_iterator cit = device.grippers().begin();
            cit != device.grippers().end(); ++cit)
        {
            JointVector_t disabledPositions(
                        CloneDisabledPositions(clone, (*cit)->getDisabledCollisions()));
            GripperPtr_t cloneGripper = Gripper::create((*cit)->name(),
                                 clone.getJointByName((*cit)->joint()->name()),
                                 (*cit)->objectPositionInJoint(),
                                 disabledPositions);
            res.push_back(cloneGripper);

        }
        return res;
    }

    void CloneCollisionPairs(hpp::model::Device &clone, const hpp::model::Device &device, Request_t type)
    {
        for(Device::CollisionPairs_t::const_iterator cit = device.collisionPairs(type).begin();
            cit != device.collisionPairs(type).end(); ++cit)
        {
            clone.addCollisionPairs(clone.getJointByName(cit->first->name()),
                                    clone.getJointByName(cit->second->name()),type);
        }
    }
    }

    // ========================================================================

    DevicePtr_t Device::clone() const
    {
      return Device::createCopy(weakPtr_.lock());
    }

    // ========================================================================

    void Device::init(const DeviceWkPtr_t& weakPtr)
    {
      weakPtr_ = weakPtr;
    }

    // ========================================================================

    void Device::initCopy(const DeviceWkPtr_t& weakPtr, const Device& device)
    {
      init(weakPtr);
      CloneJoints(*this, device);
      grippers_ = CloneGrippers(*this, device);
      setDimensionExtraConfigSpace(device.extraConfigSpace().dimension());
      collisionPairs_.clear();
      distances_.clear();
      CloneCollisionPairs(*this, device,COLLISION);
      CloneCollisionPairs(*this, device,DISTANCE);
      computeDistances();
    }

    // ========================================================================

    size_type Device::configSize () const
    {
      hppDout (info, configSize_ + extraConfigSpace_.dimension ());
      return configSize_ + extraConfigSpace_.dimension ();
    }

    // ========================================================================

    /// Size of velocity vectors
    size_type Device::numberDof () const
    {
      hppDout (info, numberDof_ + extraConfigSpace_.dimension ());
      return numberDof_ + extraConfigSpace_.dimension ();
    }

    // ========================================================================

    const ObjectVector_t& Device::obstacles (Request_t type) const
    {
      if (type == COLLISION) {
	return collisionObstacles_;
      }
      if (type == DISTANCE) {
	return distanceObstacles_;
      }
      throw std::runtime_error ("type should be either COLLISION or DISTANCE.");
    }

    // ========================================================================

    void Device::addCollisionPairs (const JointPtr_t& joint1,
				    const JointPtr_t& joint2,
				    Request_t type)
    {
      BodyPtr_t body1 = joint1->linkedBody ();
      BodyPtr_t body2 = joint2->linkedBody ();
      if (type == COLLISION) {
	// Check that collision pair has not already been added
	CollisionPair_t colPair (joint2, joint1);
	if (std::find (collisionPairs_.begin (), collisionPairs_.end (),
		       colPair) != collisionPairs_.end ()) {
	  throw std::runtime_error (std::string ("Collision pair between ") +
				    joint2->name () + std::string (" and ") +
				    joint1->name () +
				    std::string (" has already been added."));
	}
	colPair = CollisionPair_t (joint1, joint2);
	if (std::find (collisionPairs_.begin (), collisionPairs_.end (),
		       colPair) != collisionPairs_.end ()) {
	  throw std::runtime_error (std::string ("Collision pair between ") +
				    joint1->name () + std::string (" and ") +
				    joint2->name () +
				    std::string (" has already been added."));
	}
	// Add each inner object of body 1 as outer object of body 2
	const ObjectVector_t& collisionObjects =
	  body1->innerObjects (COLLISION);
	hppDout (info, "Number of collision objects in body "
		 << body1->name () << ": " << collisionObjects.size ());
	for (ObjectVector_t::const_iterator itObj1 =
	       collisionObjects.begin ();
	     itObj1 != collisionObjects.end (); ++itObj1) {
	  body2->addOuterObject (*itObj1, true, false);
	  hppDout (info, "Adding object " << (*itObj1)->name ()
		   << " to body " << body2->name ()
		   << " for collision");
	}
	// Add pair in vector
	collisionPairs_.push_back (colPair);
      }
      if (type == DISTANCE) {
	// Check that collision pair has not already been added
	CollisionPair_t colPair (joint2, joint1);
	if (std::find (distancePairs_.begin (), distancePairs_.end (),
		       colPair) != distancePairs_.end ()) {
	  throw std::runtime_error (std::string ("Distance pair between ") +
				    joint2->name () + std::string (" and ") +
				    joint1->name () +
				    std::string (" has already been added."));
	}
	colPair = CollisionPair_t (joint1, joint2);
	if (std::find (distancePairs_.begin (), distancePairs_.end (),
		       colPair) != distancePairs_.end ()) {
	  throw std::runtime_error (std::string ("Distance pair between ") +
				    joint1->name () + std::string (" and ") +
				    joint2->name () +
				    std::string (" has already been added."));
	}
	const ObjectVector_t& distanceObjects =
	  body1->innerObjects (DISTANCE);
	hppDout (info, "Number of distance objects in body "
		 << body1->name () << ": " << distanceObjects.size ());
	for (ObjectVector_t::const_iterator itObj1 =
	       distanceObjects.begin ();
	     itObj1 != distanceObjects.end (); ++itObj1) {
	  body2->addOuterObject (*itObj1, false, true);
	  hppDout (info, "Adding object " << (*itObj1)->name ()
		   << " to body " << body2->name ()
		   << " for distance");
	}
	// Add pair in vector
	distancePairs_.push_back (colPair);
      }
    }

    // ========================================================================
    void Device::removeCollisionPairs (const JointPtr_t& joint1,
				       const JointPtr_t& joint2,
				       Request_t type)
    {
      BodyPtr_t body1 = joint1->linkedBody ();
      BodyPtr_t body2 = joint2->linkedBody ();
      if (type == COLLISION) {
	// Check that the pair is referenced and remove it from the list
	CollisionPair_t colPair (joint2, joint1);
	CollisionPairs_t::iterator itPair = std::find
	  (collisionPairs_.begin (), collisionPairs_.end (), colPair);
	if (itPair == collisionPairs_.end ()) {
	  colPair = CollisionPair_t (joint1, joint2);
	  itPair = std::find (collisionPairs_.begin (),
			       collisionPairs_.end (), colPair);
	  if (itPair == collisionPairs_.end ()) {
	    throw std::runtime_error (std::string ("No collision pair between ")
				      + joint2->name () + std::string (" and ")
				      + joint1->name () +
				      std::string ("."));
	  }
	}
	collisionPairs_.erase (itPair);
	// delete each inner object of body 1 of outer objects list of body 2
	const ObjectVector_t& collisionObjects =
	  body1->innerObjects (COLLISION);
	hppDout (info, "Number of collision objects in joint "
		 << joint1->name () << ": " << collisionObjects.size ());
	for (ObjectVector_t::const_iterator itObj1 =
	       collisionObjects.begin ();
	     itObj1 != collisionObjects.end (); ++itObj1) {
	  body2->removeOuterObject (*itObj1, true, false);
	  hppDout (info, "delete object " << (*itObj1)->name ()
		   << " to body " << body2->name ()
		   << " for collision");
        }
	// Delete each inner object of body 2 of outer objects list of body 1
	const ObjectVector_t& collisionObjects2 =
	  body2->innerObjects (COLLISION);
	hppDout (info, "Number of collision objects in joint "
		 << joint2->name () << ": " << collisionObjects2.size ());
	for (ObjectVector_t::const_iterator itObj2 =
	       collisionObjects2.begin ();
	     itObj2 != collisionObjects2.end (); ++itObj2) {
	  body1->removeOuterObject (*itObj2, true, false);
	  hppDout (info, "delete object " << (*itObj2)->name ()
		   << " to body " << body1->name ()
		   << " for collision");
        }
      }
      if (type == DISTANCE) {
	// Check that the pair is referenced and remove it from the list
	CollisionPair_t colPair (joint2, joint1);
	CollisionPairs_t::iterator itPair = std::find
	  (distancePairs_.begin (), distancePairs_.end (), colPair);
	if (itPair == distancePairs_.end ()) {
	  colPair = CollisionPair_t (joint1, joint2);
	  itPair = std::find (distancePairs_.begin (),
			       distancePairs_.end (), colPair);
	  if (itPair == distancePairs_.end ()) {
	    throw std::runtime_error (std::string ("No collision pair between ")
				      + joint2->name () + std::string (" and ")
				      + joint1->name () +
				      std::string ("."));
	  }
	}
	distancePairs_.erase (itPair);
	const ObjectVector_t& distanceObjects =
	  body1->innerObjects (DISTANCE);
	hppDout (info, "Number of distance objects in joint "
		 << joint1->name () << ": " << distanceObjects.size ());
	for (ObjectVector_t::const_iterator itObj1 =
	       distanceObjects.begin ();
	     itObj1 != distanceObjects.end (); ++itObj1) {
	  body2->removeOuterObject (*itObj1, false, true);
	  hppDout (info, "delete object " << (*itObj1)->name ()
		   << " to body " << body2->name ()
		   << " for distance");
	}
        const ObjectVector_t& distanceObjects2 =
	  body2->innerObjects (DISTANCE);
	hppDout (info, "Number of distance objects in joint "
		 << joint2->name () << ": " << distanceObjects2.size ());
	for (ObjectVector_t::const_iterator itObj2 =
	       distanceObjects2.begin ();
	     itObj2 != distanceObjects2.end (); ++itObj2) {
	  body1->removeOuterObject (*itObj2, false, true);
	  hppDout (info, "delete object " << (*itObj2)->name ()
		   << " to body " << body1->name ()
		   << " for distance");
	}
      }
    }

    // ========================================================================

    const Device::CollisionPairs_t& Device::collisionPairs
    (Request_t type) const
    {
      if (type == COLLISION) {
	return collisionPairs_;
      }
      else if (type == DISTANCE) {
	return distancePairs_;
      }
      throw std::runtime_error ("type should be either COLLISION or DISTANCE.");
    }

    // ========================================================================

    ObjectIterator Device::objectIterator (Request_t type)
    {
      return ObjectIterator (*this, type);
    }


    // ========================================================================

    void Device::updateDistances ()
    {
      JointVector_t joints = getJointVector ();
      JointVector_t::size_type size = 0;
      for (JointVector_t::iterator it = joints.begin (); it != joints.end ();
	   ++it) {
	BodyPtr_t body = (*it)->linkedBody ();
	if (body) {
	  size += body->innerObjects (DISTANCE).size () *
	    body->outerObjects (DISTANCE).size ();
	}
	distances_.resize (size);
      }
    }

    // ========================================================================

    void Device::computeDistances ()
    {
      JointVector_t joints = getJointVector ();
      JointVector_t::size_type offset = 0;
      for (JointVector_t::iterator it = joints.begin (); it != joints.end ();
	   ++it) {
	BodyPtr_t body = (*it)->linkedBody ();
	if (body) {
	  body->computeDistances (distances_, offset);
	  assert (offset <= distances_.size ());
	}
      }
    }

    // ========================================================================

    bool Device::collisionTest () const
    {
      for (JointVector_t::const_iterator itJoint = jointVector_.begin ();
	   itJoint != jointVector_.end (); ++itJoint) {
	BodyPtr_t body = (*itJoint)->linkedBody ();
	if (body != 0x0) {
	  if (body->collisionTest ()) {
	    return true;
	  }
	}
      }
      return false;
    }

    // ========================================================================

    void Device::computeForwardKinematics ()
    {
      if (upToDate_) return;
      computeJointPositions ();
      if (computationFlag_ | JACOBIAN) {
	computeJointJacobians ();
      }
      if (computationFlag_ | COM) {
	computePositionCenterOfMass ();
      }
      if (computationFlag_ | COM && computationFlag_ | JACOBIAN) {
	computeJacobianCenterOfMass ();
      }
      // Update positions of bodies from position of joints.
      JointVector_t jv = getJointVector ();
      for (JointVector_t::iterator itJoint = jv.begin (); itJoint != jv.end ();
	   ++itJoint) {
	BodyPtr_t body = (*itJoint)->linkedBody ();
	if (body) {
	  const ObjectVector_t& cbv =
	    body->innerObjects (COLLISION);
	  for (ObjectVector_t::const_iterator itInner = cbv.begin ();
	       itInner != cbv.end (); ++itInner) {
	    // Compute global position if inner object
	    fcl::Transform3f globalPosition =
	      (*itJoint)->currentTransformation ()*
	      (*itInner)->positionInJointFrame ();
	    (*itInner)->fcl ()->setTransform (globalPosition);
	  }
	  const ObjectVector_t& dbv =
	    body->innerObjects (DISTANCE);
	  for (ObjectVector_t::const_iterator itInner = dbv.begin ();
	       itInner != dbv.end (); ++itInner) {
	    // Compute global position if inner object
	    fcl::Transform3f globalPosition =
	      (*itJoint)->currentTransformation ()*
	      (*itInner)->positionInJointFrame ();
	    (*itInner)->fcl ()->setTransform (globalPosition);
	  }
	}
      }
      upToDate_ = true;
      hppDout (info, *this);
    }

    // ========================================================================

    void Device::registerJoint (const JointPtr_t& joint)
    {
      jointVector_.push_back (joint);
      if(joint->configSize() > 0)
        joint->rankInConfiguration_ = configSize_;
      else
        joint->rankInConfiguration_ = configSize_ - 1 ;
      if(joint->numberDof () > 0)
        joint->rankInVelocity_ = numberDof_;
      else
        joint->rankInVelocity_ = numberDof_ - 1;
      for (size_type i = 0; i < joint->configSize(); ++i)
        jointByConfigRank_.push_back (joint);
      for (size_type i = 0; i < joint->numberDof(); ++i)
        jointByVelocityRank_.push_back (joint);
      numberDof_ += joint->numberDof ();
      configSize_ += joint->configSize ();
      resizeState (joint);
      jointByName_ [joint->name ()] = joint;
      resizeJacobians ();
      computeMass ();
    }

    void Device::resizeState (const JointPtr_t& joint)
    {
      size_type oldSize = currentConfiguration_.size ();
      size_type newSize = configSize ();
      Configuration_t q = currentConfiguration_;
      currentConfiguration_.resize (newSize);
      q0_ = Configuration_t(newSize);
      // if size of configuration increased, set last coordinates to 0
      if (newSize > oldSize) {
	currentConfiguration_.head (oldSize) = q;
	currentConfiguration_.tail (newSize - oldSize).setZero ();
	if (joint) {
	  currentConfiguration_.tail (newSize - oldSize) =
	    joint->neutralConfiguration ();
	}
      }
      oldSize = currentVelocity_.size ();
      newSize = numberDof ();
      currentVelocity_.resize (newSize);
      currentAcceleration_.resize (newSize);
      if (newSize > oldSize) {
	currentVelocity_.tail (newSize - oldSize).setZero ();
	currentAcceleration_.tail (newSize - oldSize).setZero ();
      }
    }

    void Device::rootJoint (JointPtr_t joint)
    {
      rootJoint_ = joint;
      registerJoint (joint);
      joint->robot (weakPtr_.lock ());
    }

    void Device::rootJointPosition (const Transform3f& position)
    {
      if (!rootJoint_) {
	throw std::runtime_error ("The device has no root joint.");
      }
      rootJoint_->positionInParentFrame_ = position;
      upToDate_ = false;
    }

    JointPtr_t Device::rootJoint () const
    {
      return rootJoint_;
    }

    const JointVector_t& Device::getJointVector () const
    {
      return jointVector_;
    }

    JointPtr_t Device::getJointAtConfigRank (const size_type& r) const
    {
      return jointByConfigRank_[r];
    }

    JointPtr_t Device::getJointAtVelocityRank (const size_type& r) const
    {
      return jointByVelocityRank_[r];
    }

    JointPtr_t Device::getJointByName (const std::string& name) const
    {
      JointByName_t::const_iterator it = jointByName_.find (name);
      if (it == jointByName_.end ()) {
	throw std::runtime_error ("Device " + name_ +
				  " does not have any joint named "
				  + name);
      }
      return it->second;
    }

    JointPtr_t Device::getJointByBodyName (const std::string& name) const
    {
      for (JointVector_t::const_iterator itJoint= jointVector_.begin ();
	   itJoint != jointVector_.end (); ++itJoint) {
	const JointPtr_t& joint = *itJoint;
	BodyPtr_t body = joint->linkedBody ();
	if (body && body->name () == name) {
	  return joint;
	}
      }
      throw std::runtime_error ("Device " + name_ +
				" has no joint with body of name "
				+ name);
    }

    void Device::computeJointPositions ()
    {
      if (rootJoint_) {
	rootJoint_->recursiveComputePosition (currentConfiguration_, I4);
      }
    }

    void Device::computeJointJacobians ()
    {
      if (rootJoint_) {
	rootJoint_->computeJacobian ();
	for (JointVector_t::const_iterator it = jointVector_.begin ();
	     it != jointVector_.end (); ++it) {
	}
      }
    }

    void Device::computeMass ()
    {
      mass_ = 0;
      if (rootJoint_) {
	mass_ = rootJoint_->computeMass ();
      }
    }
    void Device::computePositionCenterOfMass ()
    {
      com_.setZero ();
      if (rootJoint_) {
	rootJoint_->computeMassTimesCenterOfMass ();
	com_ = (1/mass_) * rootJoint_->massCom_;
      }
    }

    void Device::computeJacobianCenterOfMass ()
    {
      for (JointVector_t::iterator it = jointVector_.begin ();
	   it != jointVector_.end (); ++it) {
	(*it)->writeComSubjacobian (jacobianCom_, mass ());
      }
    }

    Configuration_t Device::neutralConfiguration () const
    {
      Configuration_t nc (configSize());
      const JointVector_t& jv = getJointVector ();
      for (JointVector_t::const_iterator it = jv.begin ();
          it != jv.end (); it++)
        nc.segment ((*it)->rankInConfiguration (), (*it)->configSize()) =
          (*it)->neutralConfiguration ();
      return nc;
    }

    void Device::resizeJacobians ()
    {
      jacobianCom_.resize (3, numberDof_);
      jacobianCom_.setZero ();
      for (JointVector_t::iterator itJoint = jointVector_.begin ();
	   itJoint != jointVector_.end () ; ++itJoint) {
	(*itJoint)->jacobian_.resize (6, numberDof_);
	(*itJoint)->jacobian_.setZero ();
      }
    }
    std::ostream& Device::print(std::ostream& os) const
    {
      os << "digraph G {" << std::endl;
      os << "\"Device-" << name () << "\" [shape = box label=\"Device "
	 << name () << "\\n";
      os << " Current configuration: " << currentConfiguration ().transpose ()
	 << "\\n";
      // Get position of center of mass
      hpp::model::vector3_t com = positionCenterOfMass ();
      os << "total mass " << mass() << ", COM: "
	 << com [0] <<", "<< com [1] << ", " << com [2] << "\"]" << std::endl;
      //
      // Go through joints and output each joint
      //
      hpp::model::JointPtr_t joint = rootJoint();

      if (joint) {
	os << "\"Device-" << name () << "\"->\"" << rootJoint ()->name ()
	   << "\"" << std::endl;
	os << *joint << std::endl;
      }
      os << "}" << std::endl;

      os << "Grippers:" << std::endl;
	for (Grippers_t::const_iterator it = grippers_.begin ();
	     it != grippers_.end (); ++it) {
	  (*it)->print (os); os << std::endl;
	}

      return os;
    }

    std::ostream& operator<<(std::ostream& os, const hpp::model::Device& device)
    {
      return device.print (os);
    }
  } // namespace model
} // namespace hpp
