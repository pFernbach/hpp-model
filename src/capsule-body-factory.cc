///
/// Copyright (c) 2012 CNRS
/// Authors: Antonio El Khoury
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

#include <hpp/util/debug.hh>

#include "hpp/model/capsule-body-factory.hh"

namespace hpp {
  namespace model {

    CapsuleBodyFactory::CapsuleBodyFactory ()
      : BodyFactory ()
    {
    }

    CapsuleBodyFactory::~CapsuleBodyFactory ()
    {
    }

    CapsuleBodyFactoryShPtr CapsuleBodyFactory::create ()
    {
      CapsuleBodyFactory* hppCapsuleBodyFactory = new CapsuleBodyFactory ();
      CapsuleBodyFactoryShPtr hppCapsuleBodyFactoryShPtr
	(hppCapsuleBodyFactory);
      CapsuleBodyFactoryWkPtr hppCapsuleBodyFactoryWkPtr
	= hppCapsuleBodyFactoryShPtr;

      if (hppCapsuleBodyFactory->init(hppCapsuleBodyFactoryWkPtr) != KD_OK) {
	hppDout(error," error in create() ");
	hppCapsuleBodyFactoryShPtr.reset();
      }
      return hppCapsuleBodyFactoryShPtr;
    }

    //=========================================================================

    ktStatus CapsuleBodyFactory::init (const CapsuleBodyFactoryWkPtr weakPtr)
    {
      weakPtr_ = weakPtr;
      return KD_OK;
    }

    //=========================================================================

    CkwsKCDBodyShPtr CapsuleBodyFactory::make () const
    {
      return CapsuleBody::create ("");
    }
    
  } // namespace model
} // namespace hpp
