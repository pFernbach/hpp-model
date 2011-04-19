/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include "hppModel/hppHumanoidRobot.h"
#include "hppModel/hppJoint.h"

// ==========================================================================

ChppHumanoidRobot::ChppHumanoidRobot
(CjrlRobotDynamicsObjectFactory *inObjFactory) :
  CimplDynamicRobot(), CimplHumanoidDynamicRobot(inObjFactory),
  ChppDevice()
{
}

// ==========================================================================

ChppHumanoidRobotShPtr ChppHumanoidRobot::create(std::string inName)
{
  CimplObjectFactory* objFactory = new CimplObjectFactory();
  ChppHumanoidRobot *hppDevice = new ChppHumanoidRobot(objFactory);
  ChppHumanoidRobotShPtr hppDeviceShPtr(hppDevice);

  if (hppDevice->init(hppDeviceShPtr, inName) != KD_OK) {
    hppDeviceShPtr.reset();
  }
  return hppDeviceShPtr;
}

// ==========================================================================

ChppHumanoidRobotShPtr
ChppHumanoidRobot::createCopy(const ChppHumanoidRobotShPtr& inDevice)
{
  ChppHumanoidRobot* ptr = new ChppHumanoidRobot(*inDevice);
  ChppHumanoidRobotShPtr deviceShPtr(ptr);

  if(KD_OK != ptr->init(deviceShPtr, inDevice))	{
    deviceShPtr.reset();
  }

  return deviceShPtr;
}

// ==========================================================================

CkwsDeviceShPtr ChppHumanoidRobot::clone() const
{
  return ChppHumanoidRobot::createCopy(attWeakPtr.lock());
}

// ==========================================================================

CkppComponentShPtr ChppHumanoidRobot::cloneComponent() const
{
  return ChppHumanoidRobot::createCopy(attWeakPtr.lock());
}

// ==========================================================================

bool ChppHumanoidRobot::isComponentClonable() const
{
  return true;
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppWaist()
{
  return jrlToHppJoint(waist());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppChest()
{
  return jrlToHppJoint(chest());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppLeftWrist()
{
  return jrlToHppJoint(leftWrist());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppRightWrist()
{
  return jrlToHppJoint(rightWrist());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppLeftAnkle()
{
  return jrlToHppJoint(leftAnkle());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppRightAnkle()
{
  return jrlToHppJoint(rightAnkle());
}

// ==========================================================================

ChppJoint* ChppHumanoidRobot::hppGazeJoint()
{
  return jrlToHppJoint(gazeJoint());
}

// ==========================================================================

ktStatus ChppHumanoidRobot::init(const ChppHumanoidRobotWkPtr& inDevWkPtr,
				 const std::string &inName)
{
  ktStatus success = ChppDevice::init(inDevWkPtr, inName);

  if(KD_OK == success) {
    attWeakPtr = inDevWkPtr;
  }
  return success;
}

// ==========================================================================

ktStatus ChppHumanoidRobot::init(const ChppHumanoidRobotWkPtr& inWeakPtr,
				 const ChppHumanoidRobotShPtr& inDevice)
{
  ktStatus  success = ChppDevice::init(inWeakPtr, inDevice);

  if(KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}

// Write humanoid robot in a stream
std::ostream& operator<<(std::ostream& os, ChppHumanoidRobot& inRobot)
{
  os << (ChppDevice&)inRobot << std::endl;
  if (inRobot.hppGazeJoint())
    os << "gaze joint: " << inRobot.hppGazeJoint()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppWaist())
    os << "waist joint: " << inRobot.hppWaist()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppChest())
    os << "chest joint: " << inRobot.hppChest()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppLeftWrist())
    os << "left wrist joint: " << inRobot.hppLeftWrist()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppRightWrist())
    os << "right wrist joint: " << inRobot.hppRightWrist()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppLeftAnkle())
    os << "left ankle joint: " << inRobot.hppLeftAnkle()->kppJoint()->name()
       << std::endl;
  if (inRobot.hppRightAnkle())
    os << "right ankle joint: " << inRobot.hppRightAnkle()->kppJoint()->name()
       << std::endl;

  os << "gaze origin: " << inRobot.gazeOrigin() << std::endl;
  os << "gaze direction: " << inRobot.gazeDirection() << std::endl;

  CjrlHand* hand;
  vector3d v;
  // right hand
  hand = inRobot.rightHand();
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
  hand = inRobot.leftHand();
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
  foot = inRobot.rightFoot();
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
  foot = inRobot.leftFoot();
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
