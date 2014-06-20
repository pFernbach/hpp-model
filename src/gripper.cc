///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux, Mathieu Geisert
///
///
// This file is part of hpp-model.
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model. If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/model/body.hh>
#include <fcl/math/transform.h>
#include <hpp/model/joint.hh>
#include <hpp/model/gripper.hh>

namespace hpp {
  namespace model {

      //DifferentiableFunctionPtr_t Gripper::createGrasp(HandlePtr_t& handle)
      //{
      //  return handle->createGrasp(GripperPtr_t(this));
      //}

      GripperPtr_t Gripper::clone () const
      {
        GripperPtr_t self = weakPtr_.lock ();
        return Gripper::create (self->name (),self->joint (),
                                self->objectPositionInJoint (), 
                                self->getDisabledCollisions());
      }

      std::ostream& Gripper::print (std::ostream& os) const
      {
        os << "name :" << name () << std::endl;
        os << "handle Position in joint :" << objectPositionInJoint ();
        os << "joint :" << joint ()->name () << std::endl;
        os << "disable Collisions : ";
        for (JointVector_t::const_iterator itJoint = disabledCollisions_.begin() ;
              itJoint != disabledCollisions_.end() ; itJoint++ ) {
          os << (*itJoint)->name() << "  ";
        }
        os << std::endl;
        return os;
      }


  } // namespace model
} // namespace hpp
std::ostream& operator<< (std::ostream& os,
			  const hpp::model::Gripper& gripper)
{
  return gripper.print (os);
}
