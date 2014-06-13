///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux, Mathieu Geisert
///
///
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_GRIPPER_HH
# define HPP_MANIPULATION_GRIPPER_HH

# include <fcl/math/transform.h>
# include <hpp/model/joint.hh>

namespace hpp {
  namespace model {
    /// Constraint between the position of a robot joint and of an object handle
    class HPP_MODEL_DLLAPI Gripper
    {
    public:
      /// Return a shared pointer to new instance
      /// \param joint joint of the robot that will hold handles,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      static GripperPtr_t create (const std::string& name, const JointPtr_t& joint,
				const Transform3f& handlePositionInJoint)
      {
	Gripper* ptr = new Gripper (name, joint, handlePositionInJoint);
	GripperPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Get joint that grip
      const JointPtr_t& joint () const
      {
	return joint_;
      }

      /// Set joint that grip
      void joint (const JointPtr_t& joint) 
      {  
        joint_ = joint;
      }

      /// Get handle position in the the Grippering joint
      const Transform3f& handlePositionInJoint () const
      {
	return handlePositionInJoint_;
      }
      ///get name
      const std::string& name () const
      {
	return name_;
      }
      /// Set name
      void name (const std::string& n)
      {
	name_ = n;
      }

      
      //DifferentiableFunctionPtr_t createGrasp(HandlePtr_t& handle);

      GripperPtr_t clone () const;

      virtual std::ostream& print (std::ostream& os) const;

    protected:
      /// Constructor
      /// \param joint joint of the robot that holds the handle,
      /// \param handlePositionInJoint handle position in the the grasping
      ///        joint.
      Gripper (const std::string& name, const JointPtr_t& joint,
	     const Transform3f& handlePositionInJoint) :
        name_ (name),
	joint_ (joint),
	handlePositionInJoint_ (handlePositionInJoint)
      {
      }

      void init (GripperWkPtr_t weakPtr)
      {
	weakPtr_ = weakPtr;
      }

    private:
      std::string name_;
      /// Joint of the robot that holds handles.
      JointPtr_t joint_;
      Transform3f handlePositionInJoint_;
      /// Weak pointer to itself
      GripperWkPtr_t weakPtr_;
    }; // class Gripper
  } // namespace model
} // namespace hpp
std::ostream& operator<< (std::ostream& os,
			  const hpp::model::Gripper& gripper);

#endif // HPP_MODEL_GRIPPER_HH
