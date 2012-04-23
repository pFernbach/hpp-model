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

#ifndef HPP_MODEL_BODY_FACTORY_HH
#define HPP_MODEL_BODY_FACTORY_HH

/*************************************
INCLUDE
**************************************/

#include <KineoModel/kppJointComponent.h>

#include "hpp/model/fwd.hh"
#include "hpp/model/body.hh"

namespace hpp {
  namespace model {

    /// \brief A body factory creates bodies that will can be attached
    /// to joints.
    /// 
    /// A body factory can be passed as argument at the joint
    /// initialization, the joint will then take care of creating the
    /// body (if it has not been created yet) when adding a solid
    /// component reference.
    ///
    /// \sa Smart pointers documentation:
    /// http://www.boost.org/libs/smart_ptr/smart_ptr.htm
    class BodyFactory
      : public CkppBodyFactory
    {
    public:
      /// \brief Destructor.
      virtual ~BodyFactory ();

      /// \brief Creation of a body factory.
      /// \return A shared pointer to a new body factory.
      static BodyFactoryShPtr create ();

      /// \brief Create a new body.
      virtual CkwsKCDBodyShPtr make () const;

    protected:

      /// \brief Constructor.
      BodyFactory ();

      /// \brief Initialize body factory.
      /// \param weakPtr weak pointer to itself
      ktStatus init (const BodyFactoryWkPtr weakPtr);

    private:

      /// \brief Weak pointer to itself
      BodyFactoryWkPtr weakPtr_;
    }; // class BodyFactory
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_BODY_FACTORY_HH
