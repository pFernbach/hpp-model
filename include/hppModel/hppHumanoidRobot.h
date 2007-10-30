/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPPHUMANOIDROBOT_H
#define HPPHUMANOIDROBOT_H

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "hppDevice.h"

KIT_PREDEF_CLASS(ChppHumanoidRobot);
/**
 \brief Humanoid robot with geometric and dynamic model 
 
 Derives from ChppDevice and from an implementation CimplHumanoidDynamicRobot  of CjrlHumanoidDynamicRobot.

 The creation of the device is done by ChppDevice::create(const std::string inName). This function returns a shared pointer to the newly created object.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/

class ChppHumanoidRobot : public ChppDevice, public CimplHumanoidDynamicRobot {
public:
  /**
     \name Construction, copy and destruction
     @{
  */

  ~ChppHumanoidRobot();

  /**
     \brief Clone as a CkwsDevice
  */
  CkwsDeviceShPtr clone() const;

  /**
     \brief Clone as a CkppComponent
  */
  CkppComponentShPtr cloneComponent() const;

  /**
     \brief Whether component is clonable.
     \return true
  */
  bool isComponentClonable() const;

  /**
     @}
  */

  /**
     \brief Creation of a new humanoid robot
     \return a shared pointer to the new robot
     \param inName Name of the device (is passed to CkkpDeviceComponent)
  */
  static ChppHumanoidRobotShPtr create(std::string inName);

  /**
     \brief Copy of a device
     \return A shared pointer to new device.
     \param inDevice Device to be copied.
  */
  static ChppHumanoidRobotShPtr createCopy(const ChppHumanoidRobotShPtr& inDevice);

protected:
  /**
     \brief Constructor
  */
  ChppHumanoidRobot();

  /**
     \brief Initialization.
  */

  ktStatus init(const ChppHumanoidRobotWkPtr& inWeakPtr, const std::string& inName);

  /**
     \brief Initialization with shared pointer.
  */

  ktStatus init(const ChppHumanoidRobotWkPtr& inWeakPtr, const ChppHumanoidRobotShPtr& inDevice);

private:

  /**
     \brief Store weak pointer to object.
  */
  ChppHumanoidRobotWkPtr attWeakPtr;
  
};

#endif
