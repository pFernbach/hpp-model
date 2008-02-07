/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include "hppModel/hppHumanoidRobot.h"

// ==========================================================================

ChppHumanoidRobotShPtr ChppHumanoidRobot::create(std::string inName)
{
  ChppHumanoidRobot *hppDevice = new ChppHumanoidRobot;
  ChppHumanoidRobotShPtr hppDeviceShPtr(hppDevice);

  if (hppDevice->init(hppDeviceShPtr, inName) != KD_OK) {
    hppDeviceShPtr.reset();
  }
  return hppDeviceShPtr;
}

// ==========================================================================

ChppHumanoidRobotShPtr ChppHumanoidRobot::createCopy(const ChppHumanoidRobotShPtr& inDevice)
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

ktStatus ChppHumanoidRobot::init(const ChppHumanoidRobotWkPtr& inDevWkPtr, const std::string &inName)
{
  ktStatus success = ChppDevice::init(inDevWkPtr, inName);

  if(KD_OK == success) {  
    attWeakPtr = inDevWkPtr;
  }
  return success;
}

// ==========================================================================

ktStatus ChppHumanoidRobot::init(const ChppHumanoidRobotWkPtr& inWeakPtr, const ChppHumanoidRobotShPtr& inDevice)
{
  ktStatus  success = ChppDevice::init(inWeakPtr, inDevice);

  if(KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}
