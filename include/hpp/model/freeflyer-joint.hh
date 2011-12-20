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

#ifndef HPP_MODEL_FREEFLYER_JOINT_HH
#define HPP_MODEL_FREEFLYER_JOINT_HH

#include <KineoModel/kppFreeFlyerJointComponent.h>
#include <abstract-robot-dynamics/joint.hh>
#include "hpp/model/joint.hh"

namespace hpp {
  namespace model {
    KIT_PREDEF_CLASS(FreeflyerJoint)
    ///
    /// \brief Freeflyer joint.
    ///
    /// This class implements a freeflyer joint deriving from
    /// CkppFreeFlyerJointComponent and from jrl-dynamic FreeflyerJoint
    ///
    /// Joints of this class contain inertia data as CkppDoubleProperty
    /// attributes.
    ///
    class FreeflyerJoint : public Joint,
			   public CkppFreeFlyerJointComponent

    {
    public:
      virtual ~FreeflyerJoint();

    public:
      virtual bool isComponentClonable () const
      {
	return false;
      }

      static FreeflyerJointShPtr create(const std::string& name,
					const CkitMat4& initialPosition);

      static FreeflyerJointShPtr create(const std::string& name);

      virtual void 
      fillPropertyVector(std::vector<CkppPropertyShPtr> &outPropertyVector)
	const;
      
      /// \brief Called when a property is set
      /// Update dynamic part.
      virtual bool modifiedProperty(const CkppPropertyShPtr &property);

    protected:
      FreeflyerJoint(const CkitMat4& initialPosition);
      FreeflyerJoint();
      ktStatus init (const FreeflyerJointWkPtr &weakPtr,
		     const std::string &name,
		     const CkitMat4& initialPosition);
      
    private:
      FreeflyerJointWkPtr weakPtr_;
    }; // class FreeflyerJoint
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_FREEFLYER_JOINT_HH
