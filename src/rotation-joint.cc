///
/// Copyright (c) 2011 CNRS
/// Authors: Florent Lamiraux
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

#include <iostream>
#include <sstream>
#include <string>

#include <hpp/util/debug.hh>

#include <hpp/model/types.hh>
#include "hpp/model/joint.hh"
#include "hpp/model/rotation-joint.hh"
#include "hpp/model/exception.hh"
#include "hpp/model/humanoid-robot.hh"

namespace hpp {
  namespace model {

    RotationJointShPtr RotationJoint::create(const std::string& name,
					     const CkitMat4& initialPosition)
    {
      RotationJoint *ptr = new RotationJoint(initialPosition);
      RotationJointShPtr shPtr = RotationJointShPtr(ptr);
      RotationJointWkPtr wkPtr = RotationJointWkPtr(shPtr);

      if (ptr->init(wkPtr, name, initialPosition) != KD_OK) {
	shPtr.reset();
	return shPtr;
      }
      hppDout(info, "Created rotation joint " + name);
      return shPtr;
    }

    RotationJointShPtr RotationJoint::create(const std::string& name)
    {
      CkitMat4 initialPosition;
      RotationJoint *ptr = new RotationJoint();
      RotationJointShPtr shPtr = RotationJointShPtr(ptr);
      RotationJointWkPtr wkPtr = RotationJointWkPtr(shPtr);
      if (ptr->init(wkPtr, name, initialPosition) != KD_OK) {
	shPtr.reset();
	return shPtr;
      }
      hppDout(info, "Created freeflyer joint without initial position" + name);
      return shPtr;
    }

    void RotationJoint::
    fillPropertyVector(std::vector<CkppPropertyShPtr>& outPropertyVector)
      const
    {
      CkppRotationJointComponent::fillPropertyVector(outPropertyVector);
      Joint::fillPropertyVector(outPropertyVector);
    }

    bool RotationJoint::modifiedProperty(const CkppPropertyShPtr &property)
    {
      if (!CkppRotationJointComponent::modifiedProperty(property))
	return false;
      hppDout(info,"RotationJoint::modifiedProperty: "
		<< *property);
      return true;
    }

    RotationJoint::RotationJoint(const CkitMat4& initialPosition) :
      hpp::model::Joint
      (Device::objectFactory()->createJointRotation
       (Joint::abstractMatrixFromCkitMat4(initialPosition))),
      CkppRotationJointComponent()
    {
      jointFactory_ = 0;
    }

    RotationJoint::RotationJoint() :
      hpp::model::Joint(0),
      CkppRotationJointComponent()
    {
      jointFactory_ = &impl::ObjectFactory::createJointRotation;
    }

    RotationJoint::~RotationJoint()
    {
    }

    ktStatus RotationJoint::init (const RotationJointWkPtr &weakPtr,
				  const std::string &name,
				  const CkitMat4& initialPosition)
    {
      ktStatus status = KD_OK;
      weakPtr_ = weakPtr;
      status = CkppRotationJointComponent::init(weakPtr,
						name,
						makeDefaultBodyFactory ());
      if (status == KD_ERROR) return KD_ERROR;
      status = Joint::init(weakPtr);
      if (status == KD_ERROR) return KD_ERROR;
      kwsJoint()->setCurrentPosition(initialPosition);
      jrlJoint ()->setName (name);
      return KD_OK;
    }
  } // namespace model
} // namespace hpp
