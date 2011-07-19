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
      /// \name Properties
      /// @{
      static const CkppProperty::TPropertyID GAZE_ID;
      static const std::string GAZE_STRING_ID;
      static const CkppProperty::TPropertyID LEFTANKLE_ID;
      static const std::string LEFTANKLE_STRING_ID;
      static const CkppProperty::TPropertyID RIGHTANKLE_ID;
      static const std::string RIGHTANKLE_STRING_ID;
      static const CkppProperty::TPropertyID LEFTWRIST_ID;
      static const std::string LEFTWRIST_STRING_ID;
      static const CkppProperty::TPropertyID RIGHTWRIST_ID;
      static const std::string RIGHTWRIST_STRING_ID;
      static const CkppProperty::TPropertyID WAIST_ID;
      static const std::string WAIST_STRING_ID;
      static const CkppProperty::TPropertyID CHEST_ID;
      static const std::string CHEST_STRING_ID;
      static const CkppProperty::TPropertyID GAZEORIGINX_ID;
      static const std::string GAZEORIGINX_STRING_ID;
      static const CkppProperty::TPropertyID GAZEORIGINY_ID;
      static const std::string GAZEORIGINY_STRING_ID;
      static const CkppProperty::TPropertyID GAZEORIGINZ_ID;
      static const std::string GAZEORIGINZ_STRING_ID;
      static const CkppProperty::TPropertyID GAZEDIRECTIONX_ID;
      static const std::string GAZEDIRECTIONX_STRING_ID;
      static const CkppProperty::TPropertyID GAZEDIRECTIONY_ID;
      static const std::string GAZEDIRECTIONY_STRING_ID;
      static const CkppProperty::TPropertyID GAZEDIRECTIONZ_ID;
      static const std::string GAZEDIRECTIONZ_STRING_ID;
      static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEX_ID;
      static const std::string ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID;
      static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEY_ID;
      static const std::string ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID;
      static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEZ_ID;
      static const std::string ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID;
      static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEX_ID;
      static const std::string SOLECENTERINLEFTFOOTFRAMEX_STRING_ID;
      static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEY_ID;
      static const std::string SOLECENTERINLEFTFOOTFRAMEY_STRING_ID;
      static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEZ_ID;
      static const std::string SOLECENTERINLEFTFOOTFRAMEZ_STRING_ID;
      static const CkppProperty::TPropertyID SOLELENGTH_ID;
      static const std::string SOLELENGTH_STRING_ID;
      static const CkppProperty::TPropertyID SOLEWIDTH_ID;
      static const std::string SOLEWIDTH_STRING_ID;
      static const CkppProperty::TPropertyID LEFTHANDCENTERX_ID;
      static const std::string LEFTHANDCENTERX_STRING_ID;
      static const CkppProperty::TPropertyID LEFTHANDCENTERY_ID;
      static const std::string LEFTHANDCENTERY_STRING_ID;
      static const CkppProperty::TPropertyID LEFTHANDCENTERZ_ID;
      static const std::string LEFTHANDCENTERZ_STRING_ID;
      static const CkppProperty::TPropertyID LEFTTHUMBAXISX_ID;
      static const std::string LEFTTHUMBAXISX_STRING_ID;
      static const CkppProperty::TPropertyID LEFTTHUMBAXISY_ID;
      static const std::string LEFTTHUMBAXISY_STRING_ID;
      static const CkppProperty::TPropertyID LEFTTHUMBAXISZ_ID;
      static const std::string LEFTTHUMBAXISZ_STRING_ID;
      static const CkppProperty::TPropertyID LEFTFOREFINGERAXISX_ID;
      static const std::string LEFTFOREFINGERAXISX_STRING_ID;
      static const CkppProperty::TPropertyID LEFTFOREFINGERAXISY_ID;
      static const std::string LEFTFOREFINGERAXISY_STRING_ID;
      static const CkppProperty::TPropertyID LEFTFOREFINGERAXISZ_ID;
      static const std::string LEFTFOREFINGERAXISZ_STRING_ID;
      static const CkppProperty::TPropertyID LEFTPALMNORMALX_ID;
      static const std::string LEFTPALMNORMALX_STRING_ID;
      static const CkppProperty::TPropertyID LEFTPALMNORMALY_ID;
      static const std::string LEFTPALMNORMALY_STRING_ID;
      static const CkppProperty::TPropertyID LEFTPALMNORMALZ_ID;
      static const std::string LEFTPALMNORMALZ_STRING_ID;

      /// Gaze joint name
      CkppStringPropertyShPtr gaze_;
      /// Left ankle joint name
      CkppStringPropertyShPtr leftAnkle_;
      /// Right ankle joint name
      CkppStringPropertyShPtr rightAnkle_;
      /// Left wrist joit name
      CkppStringPropertyShPtr leftWrist_;
      /// Right wrist joit name
      CkppStringPropertyShPtr rightWrist_;
      /// waist joint name
      CkppStringPropertyShPtr waist_;
      /// chest joint name
      CkppStringPropertyShPtr chest_;
      /// Gaze origin and direction
      CkppDoublePropertyShPtr gazeOriginX_;
      CkppDoublePropertyShPtr gazeOriginY_;
      CkppDoublePropertyShPtr gazeOriginZ_;
      CkppDoublePropertyShPtr gazeDirectionX_;
      CkppDoublePropertyShPtr gazeDirectionY_;
      CkppDoublePropertyShPtr gazeDirectionZ_;
      /// Ankle position in left foot frame
      CkppDoublePropertyShPtr anklePosInLeftFootFrameX_;
      CkppDoublePropertyShPtr anklePosInLeftFootFrameY_;
      CkppDoublePropertyShPtr anklePosInLeftFootFrameZ_;
      /// Sole center in local in left foot local frame
      CkppDoublePropertyShPtr soleCenterInLeftFootFrameX_;
      CkppDoublePropertyShPtr soleCenterInLeftFootFrameY_;
      CkppDoublePropertyShPtr soleCenterInLeftFootFrameZ_;
      /// Sole size
      CkppDoublePropertyShPtr soleLength_;
      CkppDoublePropertyShPtr soleWidth_;
      /// Left hand center
      CkppDoublePropertyShPtr leftHandCenterX_;
      CkppDoublePropertyShPtr leftHandCenterY_;
      CkppDoublePropertyShPtr leftHandCenterZ_;
      /// left hand thumb axis
      CkppDoublePropertyShPtr leftThumbAxisX_;
      CkppDoublePropertyShPtr leftThumbAxisY_;
      CkppDoublePropertyShPtr leftThumbAxisZ_;
      /// Left fore-finger axis
      CkppDoublePropertyShPtr leftForeFingerAxisX_;
      CkppDoublePropertyShPtr leftForeFingerAxisY_;
      CkppDoublePropertyShPtr leftForeFingerAxisZ_;
      /// Left palm normal
      CkppDoublePropertyShPtr leftPalmNormalX_;
      CkppDoublePropertyShPtr leftPalmNormalY_;
      CkppDoublePropertyShPtr leftPalmNormalZ_;
      /// \@
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
      /// \brief Define the properties of the device.
      virtual void
      fillPropertyVector(std::vector<CkppPropertyShPtr>& inOutPropertyVector)
	const;


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
      /// Object factory
      static impl::ObjectFactory objectFactory_;
    }; // class HumanoidRobot
  } // namespace model
} // namespace hpp
std::ostream& operator<<(std::ostream& os, hpp::model::HumanoidRobot& robot);
#endif // HPP_MODEL_HUMANOID_ROBOT_HH
