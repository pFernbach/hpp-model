/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/joint.hh"

namespace hpp {
  namespace model {
    HumanoidRobot::HumanoidRobot
    (CjrlRobotDynamicsObjectFactory *objFactory) :
      impl::DynamicRobot(), impl::HumanoidDynamicRobot(objFactory),
      Device()
    {
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

    Joint* HumanoidRobot::hppWaist()
    {
      return dynamic_cast<Joint*>(waist());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppChest()
    {
      return dynamic_cast<Joint*>(chest());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppLeftWrist()
    {
      return dynamic_cast<Joint*>(leftWrist());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppRightWrist()
    {
      return dynamic_cast<Joint*>(rightWrist());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppLeftAnkle()
    {
      return dynamic_cast<Joint*>(leftAnkle());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppRightAnkle()
    {
      return dynamic_cast<Joint*>(rightAnkle());
    }

    // ======================================================================

    Joint* HumanoidRobot::hppGazeJoint()
    {
      return dynamic_cast<Joint*>(gazeJoint());
    }

    // ======================================================================

    ktStatus HumanoidRobot::init(const HumanoidRobotWkPtr& inDevWkPtr,
				 const std::string &name)
    {
      ktStatus success = Device::init(inDevWkPtr, name);

      if(KD_OK == success) {
	weakPtr_ = inDevWkPtr;
      }
      return success;
    }

    // ======================================================================

    ktStatus HumanoidRobot::init(const HumanoidRobotWkPtr& weakPtr,
				 const HumanoidRobotShPtr& device)
    {
      ktStatus  success = Device::init(weakPtr, device);

      if(KD_OK == success) {
	weakPtr_ = weakPtr;
      }

      return success;
    }
  } // namespace model
} // namespace hpp

// Write humanoid robot in a stream
std::ostream& operator<<(std::ostream& os, hpp::model::HumanoidRobot& robot)
{
  os << (hpp::model::Device&)robot << std::endl;
  if (robot.hppGazeJoint())
    os << "gaze joint: " << robot.hppGazeJoint()->kppJoint()->name()
       << std::endl;
  if (robot.hppWaist())
    os << "waist joint: " << robot.hppWaist()->kppJoint()->name()
       << std::endl;
  if (robot.hppChest())
    os << "chest joint: " << robot.hppChest()->kppJoint()->name()
       << std::endl;
  if (robot.hppLeftWrist())
    os << "left wrist joint: " << robot.hppLeftWrist()->kppJoint()->name()
       << std::endl;
  if (robot.hppRightWrist())
    os << "right wrist joint: " << robot.hppRightWrist()->kppJoint()->name()
       << std::endl;
  if (robot.hppLeftAnkle())
    os << "left ankle joint: " << robot.hppLeftAnkle()->kppJoint()->name()
       << std::endl;
  if (robot.hppRightAnkle())
    os << "right ankle joint: " << robot.hppRightAnkle()->kppJoint()->name()
       << std::endl;

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
