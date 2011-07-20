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

#include "hpp/model/joint.hh"
#include "hpp/model/anchor-joint.hh"
#include "hpp/model/exception.hh"

namespace hpp {
  namespace model {

    AnchorJointShPtr AnchorJoint::create(const std::string& name,
					 const CkitMat4& initialPosition)
    {
      AnchorJoint *ptr = new AnchorJoint(initialPosition);
      AnchorJointShPtr shPtr = AnchorJointShPtr(ptr);
      AnchorJointWkPtr wkPtr = AnchorJointWkPtr(shPtr);
      
      if (ptr->init(wkPtr, name, initialPosition) != KD_OK) {
	shPtr.reset();
	return shPtr;
      }
      return shPtr;
    }

    void AnchorJoint::
    fillPropertyVector(std::vector<CkppPropertyShPtr>& outPropertyVector)
      const
    {
      CkppAnchorJointComponent::fillPropertyVector(outPropertyVector);
      Joint::fillPropertyVector(outPropertyVector);
    }

    AnchorJoint::AnchorJoint(const CkitMat4& initialPosition) :
      hpp::model::Joint(),
      CkppAnchorJointComponent(),
      impl::JointAnchor
      (Joint::abstractMatrixFromCkitMat4(initialPosition))
    {
    }

    AnchorJoint::~AnchorJoint()
    {
    }

    ktStatus AnchorJoint::init (const AnchorJointWkPtr &weakPtr,
				const std::string &name,
				const CkitMat4& initialPosition)
    {
      ktStatus status = KD_OK;
      weakPtr_ = weakPtr;
      status = CkppAnchorJointComponent::init(weakPtr, name);
      if (status == KD_ERROR) return KD_ERROR;
      status = Joint::init(weakPtr);
      if (status == KD_ERROR) return KD_ERROR;
      kwsJoint()->setCurrentPosition(initialPosition);
      return KD_OK;
    }
  } // namespace model
} // namespace hpp