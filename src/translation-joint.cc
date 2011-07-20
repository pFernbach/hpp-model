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
#include "hpp/model/translation-joint.hh"
#include "hpp/model/exception.hh"

namespace hpp {
  namespace model {

    TranslationJointShPtr TranslationJoint::create(const std::string& name,
						   const CkitMat4& initialPosition)
    {
      TranslationJoint *ptr = new TranslationJoint(initialPosition);
      TranslationJointShPtr shPtr = TranslationJointShPtr(ptr);
      TranslationJointWkPtr wkPtr = TranslationJointWkPtr(shPtr);
      
      if (ptr->init(wkPtr, name, initialPosition) != KD_OK) {
	shPtr.reset();
	return shPtr;
      }
      return shPtr;
    }

    void TranslationJoint::
    fillPropertyVector(std::vector<CkppPropertyShPtr>& outPropertyVector)
      const
    {
      CkppTranslationJointComponent::fillPropertyVector(outPropertyVector);
      Joint::fillPropertyVector(outPropertyVector);
    }

    TranslationJoint::TranslationJoint(const CkitMat4& initialPosition) :
      hpp::model::Joint(),
      CkppTranslationJointComponent(),
      impl::JointTranslation
      (Joint::abstractMatrixFromCkitMat4(initialPosition))
    {
    }

    TranslationJoint::~TranslationJoint()
    {
    }

    ktStatus TranslationJoint::init (const TranslationJointWkPtr &weakPtr,
				     const std::string &name,
				     const CkitMat4& initialPosition)
    {
      ktStatus status = KD_OK;
      weakPtr_ = weakPtr;
      status = CkppTranslationJointComponent::init(weakPtr, name);
      if (status == KD_ERROR) return KD_ERROR;
      status = Joint::init(weakPtr);
      if (status == KD_ERROR) return KD_ERROR;
      kwsJoint()->setCurrentPosition(initialPosition);
      return KD_OK;
    }
  } // namespace model
} // namespace hpp
