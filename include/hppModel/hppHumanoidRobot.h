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

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "hppModel/hppDevice.h"

/**
   \brief Default implementation of humanoid robot

*/

KIT_PREDEF_CLASS(ChppHumanoidRobot);

/**
   \brief Implementation of humanoid robot with geometric and dynamic model .

*/

class ChppHumanoidRobot : public ChppDevice, public virtual CimplHumanoidDynamicRobot
{
public:
  /**
     \name Construction, copy and destruction
     @{
  */

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


