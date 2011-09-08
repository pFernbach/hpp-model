///
/// Copyright (c) 2007, 2008, 2009, 2010, 2011 CNRS
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

#include <KineoUtility/kitNotificator.h>
#include <KineoUtility/kitNotification.h>

#include <hpp/util/debug.hh>

#include "hpp/model/exception.hh"
#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/joint.hh"
#include "hpp/model/robot-dynamics-impl.hh"

namespace hpp {
  namespace model {
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZE_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZE_STRING_ID("GAZE");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTANKLE_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTANKLE_STRING_ID("LEFTANKLE");
    const CkppProperty::TPropertyID
    HumanoidRobot::RIGHTANKLE_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::RIGHTANKLE_STRING_ID("RIGHTANKLE");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTWRIST_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTWRIST_STRING_ID("LEFTWRIST");
    const CkppProperty::TPropertyID
    HumanoidRobot::RIGHTWRIST_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::RIGHTWRIST_STRING_ID("RIGHTWRIST");
    const CkppProperty::TPropertyID
    HumanoidRobot::WAIST_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::WAIST_STRING_ID("WAIST");
    const CkppProperty::TPropertyID
    HumanoidRobot::CHEST_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::CHEST_STRING_ID("CHEST");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEORIGINX_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEORIGINX_STRING_ID("GAZEORIGINX");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEORIGINY_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEORIGINY_STRING_ID("GAZEORIGINY");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEORIGINZ_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEORIGINZ_STRING_ID("GAZEORIGINZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEDIRECTIONX_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEDIRECTIONX_STRING_ID("GAZEDIRECTIONX");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEDIRECTIONY_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEDIRECTIONY_STRING_ID("GAZEDIRECTIONY");
    const CkppProperty::TPropertyID
    HumanoidRobot::GAZEDIRECTIONZ_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::GAZEDIRECTIONZ_STRING_ID("GAZEDIRECTIONZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEX_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID
    ("ANKLEPOSINLEFTFOOTFRAMEX");
    const CkppProperty::TPropertyID
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEY_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID
    ("ANKLEPOSINLEFTFOOTFRAMEY");
    const CkppProperty::TPropertyID
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEZ_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID
    ("ANKLEPOSINLEFTFOOTFRAMEZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::SOLELENGTH_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::SOLELENGTH_STRING_ID("SOLELENGTH");
    const CkppProperty::TPropertyID
    HumanoidRobot::SOLEWIDTH_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::SOLEWIDTH_STRING_ID("SOLEWIDTH");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTHANDCENTERX_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTHANDCENTERX_STRING_ID
    ("LEFTHANDCENTERX");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTHANDCENTERY_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTHANDCENTERY_STRING_ID
    ("LEFTHANDCENTERY");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTHANDCENTERZ_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTHANDCENTERZ_STRING_ID
    ("LEFTHANDCENTERZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTTHUMBAXISX_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTTHUMBAXISX_STRING_ID("LEFTTHUMBAXISX");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTTHUMBAXISY_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTTHUMBAXISY_STRING_ID("LEFTTHUMBAXISY");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTTHUMBAXISZ_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTTHUMBAXISZ_STRING_ID("LEFTTHUMBAXISZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTFOREFINGERAXISX_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::LEFTFOREFINGERAXISX_STRING_ID("LEFTFOREFINGERAXISX");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTFOREFINGERAXISY_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::LEFTFOREFINGERAXISY_STRING_ID("LEFTFOREFINGERAXISY");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTFOREFINGERAXISZ_ID(CkppProperty::makeID());
    const std::string
    HumanoidRobot::LEFTFOREFINGERAXISZ_STRING_ID("LEFTFOREFINGERAXISZ");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTPALMNORMALX_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTPALMNORMALX_STRING_ID
    ("LEFTPALMNORMALX");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTPALMNORMALY_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTPALMNORMALY_STRING_ID
    ("LEFTPALMNORMALY");
    const CkppProperty::TPropertyID
    HumanoidRobot::LEFTPALMNORMALZ_ID(CkppProperty::makeID());
    const std::string HumanoidRobot::LEFTPALMNORMALZ_STRING_ID
    ("LEFTPALMNORMALZ");

    // ======================================================================

    HumanoidRobot::HumanoidRobot
    (CjrlRobotDynamicsObjectFactory *objFactory) :
      impl::DynamicRobot(), impl::HumanoidDynamicRobot(objFactory),
      Device(),
      gazeJoint_(),
      leftAnkleJoint_(),
      rightAnkleJoint_(),
      leftWristJoint_(),
      rightWristJoint_(),
      waistJoint_(),
      chestJoint_()
    {
      CkitNotificator::defaultNotificator()->subscribe<HumanoidRobot>
	(CkppComponent::DID_INSERT_CHILD, this,
	 &HumanoidRobot::componentDidInsertChild);
    }

    void HumanoidRobot::
    componentDidInsertChild(const CkitNotificationConstShPtr& notification)
    {
      JointShPtr parentJoint, childJoint;
      CkppComponentShPtr parent(notification->objectShPtr<CkppComponent>());
      CkppComponentShPtr child(notification->shPtrValue<CkppComponent>
			       (CkppComponent::CHILD_KEY));
      // Detect insertion of a child joint
      if ((parentJoint = KIT_DYNAMIC_PTR_CAST(Joint, parent)) &&
	  (childJoint = KIT_DYNAMIC_PTR_CAST(Joint, child))) {
	insertDynamicPart(parentJoint, childJoint);
	registerSpecificJoint(childJoint);
      }
    }

    // ======================================================================

    void HumanoidRobot::registerSpecificJoint(const JointShPtr& joint)
    {
      const std::string name = KIT_DYNAMIC_PTR_CAST(CkppComponent,
						    joint)->name();
      if (name == gaze_->value()) {
	hppDout(info, "gaze = " << name);
	impl::HumanoidDynamicRobot::gazeJoint(joint->jrlJoint());
	gazeJoint_ = joint;
      } else if (name == leftAnkle_->value()) {
	hppDout(info, "left ankle = " << name);
	impl::HumanoidDynamicRobot::leftAnkle(joint->jrlJoint());
	leftAnkleJoint_ = joint;
	// Create left foot if not already created
	if (impl::HumanoidDynamicRobot::leftFoot() == 0) {
	  CjrlFoot* foot =
	    Device::objectFactory()->createFoot(joint->jrlJoint());
	  vector3d anklePosition;
	  double length = soleLength_->value();
	  double width = soleWidth_->value();
	  anklePosition[0] = anklePosInLeftFootFrameX_->value();
	  anklePosition[1] = anklePosInLeftFootFrameY_->value();
	  anklePosition[2] = anklePosInLeftFootFrameZ_->value();
	  foot->setSoleSize(length, width);
	  foot->setAnklePositionInLocalFrame(anklePosition);
	  impl::HumanoidDynamicRobot::leftFoot(foot);
	}
      } else if (name == rightAnkle_->value()) {
	hppDout(info, "right ankle = " << name);
	impl::HumanoidDynamicRobot::rightAnkle(joint->jrlJoint());
	rightAnkleJoint_ = joint;
	// Create right foot if not already created
	if (impl::HumanoidDynamicRobot::rightFoot() == 0) {
	  CjrlFoot* foot =
	    Device::objectFactory()->createFoot(joint->jrlJoint());
	  vector3d anklePosition;
	  double length = soleLength_->value();
	  double width = soleWidth_->value();
	  anklePosition[0] = anklePosInLeftFootFrameX_->value();
	  anklePosition[1] = -anklePosInLeftFootFrameY_->value();
	  anklePosition[2] = anklePosInLeftFootFrameZ_->value();
	  foot->setSoleSize(length, width);
	  foot->setAnklePositionInLocalFrame(anklePosition);
	  impl::HumanoidDynamicRobot::rightFoot(foot);
	}
      } else if (name == leftWrist_->value()) {
	hppDout(info, "left wrist = " << name);
	impl::HumanoidDynamicRobot::leftWrist(joint->jrlJoint());
	leftWristJoint_ = joint;
	// Create left hand if not already created
	if (impl::HumanoidDynamicRobot::leftHand() == 0) {
	  CjrlHand* hand =
	    Device::objectFactory()->createHand(joint->jrlJoint());
	  vector3d center;
	  center[0] = leftHandCenterX_->value();
	  center[1] = leftHandCenterY_->value();
	  center[2] = leftHandCenterZ_->value();
	  vector3d thumbAxis;
	  thumbAxis[0] = leftThumbAxisX_->value();
	  thumbAxis[1] = leftThumbAxisY_->value();
	  thumbAxis[2] = leftThumbAxisZ_->value();
	  vector3d forefinger;
	  forefinger[0] = leftForeFingerAxisX_->value();
	  forefinger[1] = leftForeFingerAxisY_->value();
	  forefinger[2] = leftForeFingerAxisZ_->value();
	  vector3d palmNormal;
	  palmNormal[0] = leftPalmNormalX_->value();
	  palmNormal[1] = leftPalmNormalY_->value();
	  palmNormal[2] = leftPalmNormalZ_->value();
	  hand->setCenter(center);
	  hand->setThumbAxis(thumbAxis);
	  hand->setForeFingerAxis(forefinger);
	  hand->setPalmNormal(palmNormal);
	  impl::HumanoidDynamicRobot::leftHand(hand);
	}
      } else if (name == rightWrist_->value()) {
	hppDout(info, "right wrist = " << name);
	impl::HumanoidDynamicRobot::rightWrist(joint->jrlJoint());
	rightWristJoint_ = joint;
	// Create right hand
	if (impl::HumanoidDynamicRobot::rightHand() == 0) {
	  CjrlHand* hand =
	    Device::objectFactory()->createHand(joint->jrlJoint());
	  vector3d center;
	  center[0] = leftHandCenterX_->value();
	  center[1] = -leftHandCenterY_->value();
	  center[2] = leftHandCenterZ_->value();
	  vector3d thumbAxis;
	  thumbAxis[0] = leftThumbAxisX_->value();
	  thumbAxis[1] = -leftThumbAxisY_->value();
	  thumbAxis[2] = leftThumbAxisZ_->value();
	  vector3d forefinger;
	  forefinger[0] = leftForeFingerAxisX_->value();
	  forefinger[1] = -leftForeFingerAxisY_->value();
	  forefinger[2] = leftForeFingerAxisZ_->value();
	  vector3d palmNormal;
	  palmNormal[0] = leftPalmNormalX_->value();
	  palmNormal[1] = -leftPalmNormalY_->value();
	  palmNormal[2] = leftPalmNormalZ_->value();
	  hand->setCenter(center);
	  hand->setThumbAxis(thumbAxis);
	  hand->setForeFingerAxis(forefinger);
	  hand->setPalmNormal(palmNormal);
	  impl::HumanoidDynamicRobot::rightHand(hand);
	}
      } else if (name == waist_->value()) {
	hppDout(info, "waist = " << name);
	impl::HumanoidDynamicRobot::waist(joint->jrlJoint());
	waistJoint_ = joint;
      } else if (name == chest_->value()) {
	hppDout(info, "chest = " << name);
	impl::HumanoidDynamicRobot::chest(joint->jrlJoint());
	chestJoint_ = joint;
      }
    }

    // ======================================================================

    HumanoidRobotShPtr HumanoidRobot::create(std::string name)
    {
      impl::ObjectFactory* objFactory = new impl::ObjectFactory();
      HumanoidRobot *hppDevice = new HumanoidRobot(objFactory);
      HumanoidRobotShPtr hppDeviceShPtr(hppDevice);

      if (hppDevice->init(hppDeviceShPtr, name) != KD_OK) {
	hppDeviceShPtr.reset();
      }
      return hppDeviceShPtr;
    }

    // ======================================================================

    HumanoidRobotShPtr
    HumanoidRobot::createCopy(const HumanoidRobotShPtr& device)
    {
      HumanoidRobot* ptr = new HumanoidRobot(*device);
      HumanoidRobotShPtr deviceShPtr(ptr);

      if(KD_OK != ptr->init(deviceShPtr, device))	{
	deviceShPtr.reset();
      }

      return deviceShPtr;
    }

    // ======================================================================

    CkwsDeviceShPtr HumanoidRobot::clone() const
    {
      return HumanoidRobot::createCopy(weakPtr_.lock());
    }

    // ======================================================================

    CkppComponentShPtr HumanoidRobot::cloneComponent() const
    {
      return HumanoidRobot::createCopy(weakPtr_.lock());
    }

    // ======================================================================

    bool HumanoidRobot::isComponentClonable() const
    {
      return true;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppWaist()
    {
      return waistJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppChest()
    {
      return chestJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppLeftWrist()
    {
      return leftWristJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppRightWrist()
    {
      return rightWristJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppLeftAnkle()
    {
      return leftAnkleJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppRightAnkle()
    {
      return rightAnkleJoint_;
    }

    // ======================================================================

    JointShPtr HumanoidRobot::hppGazeJoint()
    {
      return gazeJoint_;
    }

    // ======================================================================

    ktStatus HumanoidRobot::init(const HumanoidRobotWkPtr& weakPtr,
				 const std::string &name)
    {
      // Initialize properties
      CkppComponentShPtr component = weakPtr.lock();
      gaze_ = CkppStringProperty::create("GAZE", component,
					 GAZE_ID,
					 GAZE_STRING_ID);
      if (!gaze_) {
	throw Exception("Failed to initialize GAZE property");
      }

      leftAnkle_ = CkppStringProperty::create("LEFTANKLE", component,
					      LEFTANKLE_ID,
					      LEFTANKLE_STRING_ID);
      if (!leftAnkle_) {
	throw Exception("Failed to initialize LEFTANKLE property");
      }

      rightAnkle_ = CkppStringProperty::create("RIGHTANKLE", component,
					       RIGHTANKLE_ID,
					       RIGHTANKLE_STRING_ID);
      if (!rightAnkle_) {
	throw Exception("Failed to initialize RIGHTANKLE property");
      }

      leftWrist_ = CkppStringProperty::create("LEFTWRIST", component,
					      LEFTWRIST_ID,
					      LEFTWRIST_STRING_ID);
      if (!leftWrist_) {
	throw Exception("Failed to initialize LEFTWRIST property");
      }

      rightWrist_ = CkppStringProperty::create("RIGHTWRIST", component,
					       RIGHTWRIST_ID,
					       RIGHTWRIST_STRING_ID);
      if (!rightWrist_) {
	throw Exception("Failed to initialize RIGHTWRIST property");
      }

      waist_ = CkppStringProperty::create("WAIST", component,
					  WAIST_ID,
					  WAIST_STRING_ID);
      if (!waist_) {
	throw Exception("Failed to initialize WAIST property");
      }

      chest_ = CkppStringProperty::create("CHEST", component,
					  CHEST_ID,
					  CHEST_STRING_ID);
      if (!chest_) {
	throw Exception("Failed to initialize CHEST property");
      }

      gazeOriginX_ =
	CkppDoubleProperty::create("GAZEORIGINX", component,
				   GAZEORIGINX_ID,
				   GAZEORIGINX_STRING_ID);
      if (!gazeOriginX_) {
	throw Exception("Failed to initialize GAZEORIGINX property");
      }

      gazeOriginY_ =
	CkppDoubleProperty::create("GAZEORIGINY", component,
				   GAZEORIGINY_ID,
				   GAZEORIGINY_STRING_ID);
      if (!gazeOriginY_) {
	throw Exception("Failed to initialize GAZEORIGINY property");
      }

      gazeOriginZ_ =
	CkppDoubleProperty::create("GAZEORIGINZ", component,
				   GAZEORIGINZ_ID,
				   GAZEORIGINZ_STRING_ID);
      if (!gazeOriginZ_) {
	throw Exception("Failed to initialize GAZEORIGINZ property");
      }

      gazeDirectionX_ =
	CkppDoubleProperty::create("GAZEDIRECTIONX", component,
				   GAZEDIRECTIONX_ID,
				   GAZEDIRECTIONX_STRING_ID);
      if (!gazeDirectionX_) {
	throw Exception("Failed to initialize GAZEDIRECTIONX property");
      }

      gazeDirectionY_ =
	CkppDoubleProperty::create("GAZEDIRECTIONY", component,
				   GAZEDIRECTIONY_ID,
				   GAZEDIRECTIONY_STRING_ID);
      if (!gazeDirectionY_) {
	throw Exception("Failed to initialize GAZEDIRECTIONY property");
      }

      gazeDirectionZ_ =
	CkppDoubleProperty::create("GAZEDIRECTIONZ", component,
				   GAZEDIRECTIONZ_ID,
				   GAZEDIRECTIONZ_STRING_ID);
      if (!gazeDirectionZ_) {
	throw Exception("Failed to initialize GAZEDIRECTIONZ property");
      }

      anklePosInLeftFootFrameX_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEX", component,
				   ANKLEPOSINLEFTFOOTFRAMEX_ID,
				   ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID);
      if (!anklePosInLeftFootFrameX_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEX property");
      }

      anklePosInLeftFootFrameY_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEY", component,
				   ANKLEPOSINLEFTFOOTFRAMEY_ID,
				   ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID);
      if (!anklePosInLeftFootFrameY_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEY property");
      }

      anklePosInLeftFootFrameZ_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEZ", component,
				   ANKLEPOSINLEFTFOOTFRAMEZ_ID,
				   ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID);
      if (!anklePosInLeftFootFrameZ_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEZ property");
      }

      soleLength_ = CkppDoubleProperty::create("SOLELENGTH", component,
					       SOLELENGTH_ID,
					       SOLELENGTH_STRING_ID);
      if (!soleLength_) {
	throw Exception("Failed to initialize SOLELENGTH property");
      }

      soleWidth_ = CkppDoubleProperty::create("SOLEWIDTH", component,
					      SOLEWIDTH_ID,
					      SOLEWIDTH_STRING_ID);
      if (!soleWidth_) {
	throw Exception("Failed to initialize SOLEWIDTH property");
      }

      leftHandCenterX_ =
	CkppDoubleProperty::create("LEFTHANDCENTERX", component,
				   LEFTHANDCENTERX_ID,
				   LEFTHANDCENTERX_STRING_ID);
      if (!leftHandCenterX_) {
	throw Exception("Failed to initialize LEFTHANDCENTERX property");
      }

      leftHandCenterY_ =
	CkppDoubleProperty::create("LEFTHANDCENTERY", component,
				   LEFTHANDCENTERY_ID,
				   LEFTHANDCENTERY_STRING_ID);
      if (!leftHandCenterY_) {
	throw Exception("Failed to initialize LEFTHANDCENTERY property");
      }

      leftHandCenterZ_ =
	CkppDoubleProperty::create("LEFTHANDCENTERZ", component,
				   LEFTHANDCENTERZ_ID,
				   LEFTHANDCENTERZ_STRING_ID);
      if (!leftHandCenterZ_) {
	throw Exception("Failed to initialize LEFTHANDCENTERZ property");
      }

      leftThumbAxisX_ = CkppDoubleProperty::create("LEFTTHUMBAXISX", component,
						   LEFTTHUMBAXISX_ID,
						   LEFTTHUMBAXISX_STRING_ID);
      if (!leftThumbAxisX_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISX property");
      }

      leftThumbAxisY_ = CkppDoubleProperty::create("LEFTTHUMBAXISY", component,
						   LEFTTHUMBAXISY_ID,
						   LEFTTHUMBAXISY_STRING_ID);
      if (!leftThumbAxisY_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISY property");
      }

      leftThumbAxisZ_ = CkppDoubleProperty::create("LEFTTHUMBAXISZ", component,
						   LEFTTHUMBAXISZ_ID,
						   LEFTTHUMBAXISZ_STRING_ID);
      if (!leftThumbAxisZ_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISZ property");
      }

      leftForeFingerAxisX_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISX", component,
				   LEFTFOREFINGERAXISX_ID,
				   LEFTFOREFINGERAXISX_STRING_ID);
      if (!leftForeFingerAxisX_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISX property");
      }

      leftForeFingerAxisY_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISY", component,
				   LEFTFOREFINGERAXISY_ID,
				   LEFTFOREFINGERAXISY_STRING_ID);
      if (!leftForeFingerAxisY_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISY property");
      }

      leftForeFingerAxisZ_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISZ", component,
				   LEFTFOREFINGERAXISZ_ID,
				   LEFTFOREFINGERAXISZ_STRING_ID);
      if (!leftForeFingerAxisZ_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISZ property");
      }

      leftPalmNormalX_ =
	CkppDoubleProperty::create("LEFTPALMNORMALX", component,
				   LEFTPALMNORMALX_ID,
				   LEFTPALMNORMALX_STRING_ID);
      if (!leftPalmNormalX_) {
	throw Exception("Failed to initialize LEFTPALMNORMALX property");
      }

      leftPalmNormalY_ =
	CkppDoubleProperty::create("LEFTPALMNORMALY", component,
				   LEFTPALMNORMALY_ID,
				   LEFTPALMNORMALY_STRING_ID);
      if (!leftPalmNormalY_) {
	throw Exception("Failed to initialize LEFTPALMNORMALY property");
      }

      leftPalmNormalZ_ =
	CkppDoubleProperty::create("LEFTPALMNORMALZ", component,
				   LEFTPALMNORMALZ_ID,
				   LEFTPALMNORMALZ_STRING_ID);
      if (!leftPalmNormalZ_) {
	throw Exception("Failed to initialize LEFTPALMNORMALZ property");
      }

      ktStatus success = Device::init(weakPtr, name);

      if(KD_OK == success) {
	weakPtr_ = weakPtr;
      }
      else {
	hppDout(error, "failed to init Device");
      }
      return success;
    }

    // ======================================================================

    void
    HumanoidRobot::fillPropertyVector(std::vector<CkppPropertyShPtr>&
				      inOutPropertyVector) const
    {
      CkppDeviceComponent::fillPropertyVector(inOutPropertyVector);
      inOutPropertyVector.push_back(gaze_);
      inOutPropertyVector.push_back(leftAnkle_);
      inOutPropertyVector.push_back(rightAnkle_);
      inOutPropertyVector.push_back(leftWrist_);
      inOutPropertyVector.push_back(rightWrist_);
      inOutPropertyVector.push_back(waist_);
      inOutPropertyVector.push_back(chest_);
      inOutPropertyVector.push_back(gazeOriginX_);
      inOutPropertyVector.push_back(gazeOriginY_);
      inOutPropertyVector.push_back(gazeOriginZ_);
      inOutPropertyVector.push_back(gazeDirectionX_);
      inOutPropertyVector.push_back(gazeDirectionY_);
      inOutPropertyVector.push_back(gazeDirectionZ_);
      inOutPropertyVector.push_back(anklePosInLeftFootFrameX_);
      inOutPropertyVector.push_back(anklePosInLeftFootFrameY_);
      inOutPropertyVector.push_back(anklePosInLeftFootFrameZ_);
      inOutPropertyVector.push_back(soleLength_);
      inOutPropertyVector.push_back(soleWidth_);
      inOutPropertyVector.push_back(leftHandCenterX_);
      inOutPropertyVector.push_back(leftHandCenterY_);
      inOutPropertyVector.push_back(leftHandCenterZ_);
      inOutPropertyVector.push_back(leftThumbAxisX_);
      inOutPropertyVector.push_back(leftThumbAxisY_);
      inOutPropertyVector.push_back(leftThumbAxisZ_);
      inOutPropertyVector.push_back(leftForeFingerAxisX_);
      inOutPropertyVector.push_back(leftForeFingerAxisY_);
      inOutPropertyVector.push_back(leftForeFingerAxisZ_);
      inOutPropertyVector.push_back(leftPalmNormalX_);
      inOutPropertyVector.push_back(leftPalmNormalY_);
      inOutPropertyVector.push_back(leftPalmNormalZ_);
    }

    // ======================================================================

    void HumanoidRobot::updateProperty(const CkppPropertyShPtr& property)
    {
      hppDout(info,"HumanoidRobot::updateProperty: "
		<< *property);
    }

    // ======================================================================

    bool HumanoidRobot::modifiedProperty(const CkppPropertyShPtr &property)
    {
      if (!CkppDeviceComponent::modifiedProperty(property)) return false;

      vector3d	go = gazeOrigin();
      vector3d gd = gazeDirection();

      if (property == gazeOriginX_) {
	go(0) = gazeOriginX_->value();
	return true;
      }
      else if (property == gazeOriginY_) {
	go(1) = gazeOriginY_->value();
	return true;
      }
      else if (property == gazeOriginZ_) {
	go(2) = gazeOriginZ_->value();
	return true;
      }
      else if (property == gazeDirectionX_) {
	gd(0) = gazeDirectionX_->value();
	return true;
      }
      else if (property == gazeDirectionY_) {
	gd(1) = gazeDirectionY_->value();
	return true;
      }
      else if (property == gazeDirectionZ_) {
	gd(2) = gazeDirectionZ_->value();
	return true;
      }
      hppDout(info,"HumanoidRobot::modifiedProperty: "
		<< *property);
      return true;
    }

    // ======================================================================

    ktStatus HumanoidRobot::init(const HumanoidRobotWkPtr& weakPtr,
				 const HumanoidRobotShPtr& device)
    {
      ktStatus  success = Device::init(weakPtr, device);

      if(KD_OK == success) {
	weakPtr_ = weakPtr;
      }

      // Initialize properties
      CkppComponentShPtr component = weakPtr.lock();
      gaze_ = CkppStringProperty::create("GAZE", component,
					 GAZE_ID,
					 GAZE_STRING_ID);
      if (!gaze_) {
	throw Exception("Failed to initialize GAZE property");
      }

      leftAnkle_ = CkppStringProperty::create("LEFTANKLE", component,
					      LEFTANKLE_ID,
					      LEFTANKLE_STRING_ID);
      if (!leftAnkle_) {
	throw Exception("Failed to initialize LEFTANKLE property");
      }

      rightAnkle_ = CkppStringProperty::create("RIGHTANKLE", component,
					       RIGHTANKLE_ID,
					       RIGHTANKLE_STRING_ID);
      if (!rightAnkle_) {
	throw Exception("Failed to initialize RIGHTANKLE property");
      }

      leftWrist_ = CkppStringProperty::create("LEFTWRIST", component,
					      LEFTWRIST_ID,
					      LEFTWRIST_STRING_ID);
      if (!leftWrist_) {
	throw Exception("Failed to initialize LEFTWRIST property");
      }

      rightWrist_ = CkppStringProperty::create("RIGHTWRIST", component,
					       RIGHTWRIST_ID,
					       RIGHTWRIST_STRING_ID);
      if (!rightWrist_) {
	throw Exception("Failed to initialize RIGHTWRIST property");
      }

      waist_ = CkppStringProperty::create("WAIST", component,
					  WAIST_ID,
					  WAIST_STRING_ID);
      if (!waist_) {
	throw Exception("Failed to initialize WAIST property");
      }

      chest_ = CkppStringProperty::create("CHEST", component,
					  CHEST_ID,
					  CHEST_STRING_ID);
      if (!chest_) {
	throw Exception("Failed to initialize CHEST property");
      }

      gazeOriginX_ =
	CkppDoubleProperty::create("GAZEORIGINX", component,
				   GAZEORIGINX_ID,
				   GAZEORIGINX_STRING_ID);
      if (!gazeOriginX_) {
	throw Exception("Failed to initialize GAZEORIGINX property");
      }

      gazeOriginY_ =
	CkppDoubleProperty::create("GAZEORIGINY", component,
				   GAZEORIGINY_ID,
				   GAZEORIGINY_STRING_ID);
      if (!gazeOriginY_) {
	throw Exception("Failed to initialize GAZEORIGINY property");
      }

      gazeOriginZ_ =
	CkppDoubleProperty::create("GAZEORIGINZ", component,
				   GAZEORIGINZ_ID,
				   GAZEORIGINZ_STRING_ID);
      if (!gazeOriginZ_) {
	throw Exception("Failed to initialize GAZEORIGINZ property");
      }

      gazeDirectionX_ =
	CkppDoubleProperty::create("GAZEDIRECTIONX", component,
				   GAZEDIRECTIONX_ID,
				   GAZEDIRECTIONX_STRING_ID);
      if (!gazeDirectionX_) {
	throw Exception("Failed to initialize GAZEDIRECTIONX property");
      }

      gazeDirectionY_ =
	CkppDoubleProperty::create("GAZEDIRECTIONY", component,
				   GAZEDIRECTIONY_ID,
				   GAZEDIRECTIONY_STRING_ID);
      if (!gazeDirectionY_) {
	throw Exception("Failed to initialize GAZEDIRECTIONY property");
      }

      gazeDirectionZ_ =
	CkppDoubleProperty::create("GAZEDIRECTIONZ", component,
				   GAZEDIRECTIONZ_ID,
				   GAZEDIRECTIONZ_STRING_ID);
      if (!gazeDirectionZ_) {
	throw Exception("Failed to initialize GAZEDIRECTIONZ property");
      }

      anklePosInLeftFootFrameX_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEX", component,
				   ANKLEPOSINLEFTFOOTFRAMEX_ID,
				   ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID);
      if (!anklePosInLeftFootFrameX_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEX property");
      }

      anklePosInLeftFootFrameY_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEY", component,
				   ANKLEPOSINLEFTFOOTFRAMEY_ID,
				   ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID);
      if (!anklePosInLeftFootFrameY_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEY property");
      }

      anklePosInLeftFootFrameZ_ =
	CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEZ", component,
				   ANKLEPOSINLEFTFOOTFRAMEZ_ID,
				   ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID);
      if (!anklePosInLeftFootFrameZ_) {
	throw Exception
	  ("Failed to initialize ANKLEPOSINLEFTFOOTFRAMEZ property");
      }

      soleLength_ = CkppDoubleProperty::create("SOLELENGTH", component,
					       SOLELENGTH_ID,
					       SOLELENGTH_STRING_ID);
      if (!soleLength_) {
	throw Exception("Failed to initialize SOLELENGTH property");
      }

      soleWidth_ = CkppDoubleProperty::create("SOLEWIDTH", component,
					      SOLEWIDTH_ID,
					      SOLEWIDTH_STRING_ID);
      if (!soleWidth_) {
	throw Exception("Failed to initialize SOLEWIDTH property");
      }

      leftHandCenterX_ =
	CkppDoubleProperty::create("LEFTHANDCENTERX", component,
				   LEFTHANDCENTERX_ID,
				   LEFTHANDCENTERX_STRING_ID);
      if (!leftHandCenterX_) {
	throw Exception("Failed to initialize LEFTHANDCENTERX property");
      }

      leftHandCenterY_ =
	CkppDoubleProperty::create("LEFTHANDCENTERY", component,
				   LEFTHANDCENTERY_ID,
				   LEFTHANDCENTERY_STRING_ID);
      if (!leftHandCenterY_) {
	throw Exception("Failed to initialize LEFTHANDCENTERY property");
      }

      leftHandCenterZ_ =
	CkppDoubleProperty::create("LEFTHANDCENTERZ", component,
				   LEFTHANDCENTERZ_ID,
				   LEFTHANDCENTERZ_STRING_ID);
      if (!leftHandCenterZ_) {
	throw Exception("Failed to initialize LEFTHANDCENTERZ property");
      }

      leftThumbAxisX_ = CkppDoubleProperty::create("LEFTTHUMBAXISX", component,
						   LEFTTHUMBAXISX_ID,
						   LEFTTHUMBAXISX_STRING_ID);
      if (!leftThumbAxisX_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISX property");
      }

      leftThumbAxisY_ = CkppDoubleProperty::create("LEFTTHUMBAXISY", component,
						   LEFTTHUMBAXISY_ID,
						   LEFTTHUMBAXISY_STRING_ID);
      if (!leftThumbAxisY_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISY property");
      }

      leftThumbAxisZ_ = CkppDoubleProperty::create("LEFTTHUMBAXISZ", component,
						   LEFTTHUMBAXISZ_ID,
						   LEFTTHUMBAXISZ_STRING_ID);
      if (!leftThumbAxisZ_) {
	throw Exception("Failed to initialize LEFTTHUMBAXISZ property");
      }

      leftForeFingerAxisX_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISX", component,
				   LEFTFOREFINGERAXISX_ID,
				   LEFTFOREFINGERAXISX_STRING_ID);
      if (!leftForeFingerAxisX_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISX property");
      }

      leftForeFingerAxisY_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISY", component,
				   LEFTFOREFINGERAXISY_ID,
				   LEFTFOREFINGERAXISY_STRING_ID);
      if (!leftForeFingerAxisY_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISY property");
      }

      leftForeFingerAxisZ_ =
	CkppDoubleProperty::create("LEFTFOREFINGERAXISZ", component,
				   LEFTFOREFINGERAXISZ_ID,
				   LEFTFOREFINGERAXISZ_STRING_ID);
      if (!leftForeFingerAxisZ_) {
	throw Exception("Failed to initialize LEFTFOREFINGERAXISZ property");
      }

      leftPalmNormalX_ =
	CkppDoubleProperty::create("LEFTPALMNORMALX", component,
				   LEFTPALMNORMALX_ID,
				   LEFTPALMNORMALX_STRING_ID);
      if (!leftPalmNormalX_) {
	throw Exception("Failed to initialize LEFTPALMNORMALX property");
      }

      leftPalmNormalY_ =
	CkppDoubleProperty::create("LEFTPALMNORMALY", component,
				   LEFTPALMNORMALY_ID,
				   LEFTPALMNORMALY_STRING_ID);
      if (!leftPalmNormalY_) {
	throw Exception("Failed to initialize LEFTPALMNORMALY property");
      }

      leftPalmNormalZ_ =
	CkppDoubleProperty::create("LEFTPALMNORMALZ", component,
				   LEFTPALMNORMALZ_ID,
				   LEFTPALMNORMALZ_STRING_ID);
      if (!leftPalmNormalZ_) {
	throw Exception("Failed to initialize LEFTPALMNORMALZ property");
      }
      return success;
    }

    bool HumanoidRobot::initialize ()
    {
      hppDout(info, "");
      Device::initialize();
      typedef std::vector< CkppJointComponentShPtr > vector_t;
      vector_t jointVector;
      getJointComponentVector(jointVector);
      for (vector_t::iterator it = jointVector.begin();
	   it != jointVector.end(); it++) {
	JointShPtr joint = KIT_DYNAMIC_PTR_CAST(Joint, *it);
	if (joint == 0) {
	  throw Exception("Joint is not of type hpp::model::Joint.");
	}
	registerSpecificJoint(joint);
      }
    }
  } // namespace model
} // namespace hpp

// Write humanoid robot in a stream
std::ostream& operator<<(std::ostream& os, hpp::model::HumanoidRobot& robot)
{
  os << (hpp::model::Device&)robot << std::endl;
  if (robot.hppGazeJoint()) {
    os << "gaze joint: " << robot.hppGazeJoint()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no gaze joint"  << std::endl;
  }
  if (robot.hppWaist()) {
    os << "waist joint: " << robot.hppWaist()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no waist joint"  << std::endl;
  }
  if (robot.hppChest()) {
    os << "chest joint: " << robot.hppChest()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no chest joint"  << std::endl;
  }
  if (robot.hppLeftWrist()) {
    os << "left wrist joint: " << robot.hppLeftWrist()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no left wrist"  << std::endl;
  }
  if (robot.hppRightWrist()) {
    os << "right wrist joint: " << robot.hppRightWrist()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no right wrist"  << std::endl;
  }
  if (robot.hppLeftAnkle()) {
    os << "left ankle joint: " << robot.hppLeftAnkle()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no left ankle"  << std::endl;
  }
  if (robot.hppRightAnkle()) {
    os << "right ankle joint: " << robot.hppRightAnkle()->kppJoint()->name()
       << std::endl;
  } else {
    os << "no right ankle"  << std::endl;
  }

  os << "gaze origin: " << robot.gazeOrigin() << std::endl;
  os << "gaze direction: " << robot.gazeDirection() << std::endl;

  CjrlHand* hand;
  vector3d v;
  // right hand
  hand = robot.rightHand();
  if (hand) {
    os << "right hand" << std::endl;
    hand->getCenter(v);
    os << "  center: " << v << std::endl;
    hand->getThumbAxis(v);
    os << "  thumb axis: " << v << std::endl;
    hand->getForeFingerAxis(v);
    os << "  forefinger axis: " << v << std::endl;
    hand->getPalmNormal(v);
    os << "  palm normal: " << v << std::endl;
  } else {
    os << "no right hand" << std::endl;
  }
  hand = robot.leftHand();
  if (hand) {
    os << "left hand" << std::endl;
    hand->getCenter(v);
    os << "  center: " << v << std::endl;
    hand->getThumbAxis(v);
    os << "  thumb axis: " << v << std::endl;
    hand->getForeFingerAxis(v);
    os << "  forefinger axis: " << v << std::endl;
    hand->getPalmNormal(v);
    os << "  palm normal: " << v << std::endl;
  } else {
    os << "no left hand" << std::endl;
  }

  CjrlFoot* foot;
  double soleLength, soleWidth;
  // right foot
  foot = robot.rightFoot();
  if (foot) {
    os << "right foot" << std::endl;
    foot->getSoleSize(soleLength, soleWidth);
    os << "  sole length: " << soleLength << std::endl;
    os << "  sole width: " << soleWidth << std::endl;
    foot->getAnklePositionInLocalFrame(v);
    os << "  ankle position in local frame: " << v << std::endl;
  } else {
    os << "no right foot" << std::endl;
  }

  // left foot
  foot = robot.leftFoot();
  if (foot) {
    os << "left foot" << std::endl;
    foot->getSoleSize(soleLength, soleWidth);
    os << "  sole length: " << soleLength << std::endl;
    os << "  sole width: " << soleWidth << std::endl;
    foot->getAnklePositionInLocalFrame(v);
    os << "  ankle position in local frame: " << v << std::endl;
  } else {
    os << "no left foot" << std::endl;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const CkppProperty& property)
{
  os << property.name();
  try {
    const CkppStringProperty& sp =
      dynamic_cast<const CkppStringProperty&> (property);
    os << " = " << sp.value();
  } catch (...) {}
  try {
    const CkppDoubleProperty& dp =
      dynamic_cast<const CkppDoubleProperty&> (property);
    os << " = " << dp.value();
  } catch (...) {}
  return os;
}
