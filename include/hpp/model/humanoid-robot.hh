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

#ifndef HPP_MODEL_HUMANOID_ROBOT_HH
#define HPP_MODEL_HUMANOID_ROBOT_HH

#include <KineoWorks2/kwsInterface.h>
#include <KineoUtility/kitInterface.h>
#include <KineoModel/kppDeviceComponent.h>

#include <kcd2/kcdInterface.h>
#include <kwsKcd2/kwsKCDBody.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

#include "hpp/model/device.hh"

namespace hpp {
  namespace model {
    /// \brief Implementation of humanoid robot with geometric and dynamic model .
    class HumanoidRobot : public virtual impl::HumanoidDynamicRobot,
			      public Device
    {
    public:
      /// \name Construction, copy and destruction
      /// @{
      ///
      /// \brief Clone as a CkwsDevice
      CkwsDeviceShPtr clone() const;

      /// \brief Clone as a CkppComponent
      CkppComponentShPtr cloneComponent() const;

      /// \brief Whether component is clonable.
      /// \return true
      bool isComponentClonable() const;

      ///
      /// @}
      ///

      /// \name Joints specific to humanoid robots
      ///

      /// \brief Get Joint corresponding to the waist.
      Joint* hppWaist();

      /// \brief Get Joint corresponding to the chest.
      Joint* hppChest();

      /// \brief Get Joint corresponding to the left wrist.
      Joint* hppLeftWrist();

      /// \brief Get Joint corresponding to the right wrist.
      Joint* hppRightWrist();

      /// \brief Get Joint corresponding to the left ankle.
      Joint* hppLeftAnkle();

      /// \brief Get Joint corresponding to the right ankle.
      Joint* hppRightAnkle();

      /// \brief Get gaze joint
      Joint* hppGazeJoint();

      /// @}

      /// \brief Creation of a new humanoid robot
      /// \return a shared pointer to the new robot
      /// \param name Name of the device (is passed to CkkpDeviceComponent)
      static HumanoidRobotShPtr create(std::string name);

      /// \brief Copy of a device
      /// \return A shared pointer to new device.
      /// \param device Device to be copied.
      static HumanoidRobotShPtr
      createCopy(const HumanoidRobotShPtr& device);

    protected:
      /// \brief Constructor
      /// \param objFactory object factory
      /// \note Constructor of impl::HumanoidDynamicRobot takes as input
      /// a factory object.
      HumanoidRobot(CjrlRobotDynamicsObjectFactory* objFactory);

      /// \brief Initialization.
      /// \param weakPtr Weak pointer to this object.
      /// \param name Name of the object.
      ktStatus
      init(const HumanoidRobotWkPtr& weakPtr, const std::string& name);

      /// \brief Initialization for copy.
      /// \param weakPtr Weak pointer to this object
      /// \param device Reference to the source object.
      ktStatus init(const HumanoidRobotWkPtr& weakPtr,
		    const HumanoidRobotShPtr& device);

    private:
      /// \brief Store weak pointer to object.
      HumanoidRobotWkPtr weakPtr_;
    }; // class HumanoidRobot
  } // namespace model
} // namespace hpp
std::ostream& operator<<(std::ostream& os, hpp::model::HumanoidRobot& robot);
#endif // HPP_MODEL_HUMANOID_ROBOT_HH
