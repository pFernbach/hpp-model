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

#include "jrl/mal/matrixabstractlayer.hh"
#include "abstract-robot-dynamics/humanoid-dynamic-robot.hh"
#include <abstract-robot-dynamics/deprecated.hh>
#include "hppModel/hppDevice.h"

/**
   \brief Default implementation of humanoid robot

*/

KIT_PREDEF_CLASS(ChppHumanoidRobot);

/**
   \brief Implementation of humanoid robot with geometric and dynamic model .

*/

class ChppHumanoidRobot : public virtual CimplHumanoidDynamicRobot,
  public ChppDevice
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
     \name Joints specific to humanoid robots
  */

  /**
     \brief Get ChppJoint corresponding to the waist.
  */
  ChppJoint* hppWaist();

  /**
     \brief Get ChppJoint corresponding to the chest.
  */
  ChppJoint* hppChest();

  /**
     \brief Get ChppJoint corresponding to the left wrist.
  */
  ChppJoint* hppLeftWrist();

  /**
     \brief Get ChppJoint corresponding to the right wrist.
  */
  ChppJoint* hppRightWrist();

  /**
     \brief Get ChppJoint corresponding to the left ankle.
  */
  ChppJoint* hppLeftAnkle();

  /**
     \brief Get ChppJoint corresponding to the right ankle.
  */
  ChppJoint* hppRightAnkle();

  /**
     \brief Get gaze joint
  */
  ChppJoint* hppGazeJoint();

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
  static ChppHumanoidRobotShPtr
    createCopy(const ChppHumanoidRobotShPtr& inDevice);

protected:
  /**
     \brief Constructor
     \param inObjFactory factory that builds implementation of the Pimple in CimplHumanoidDynamicRobot
  */
  ChppHumanoidRobot(CjrlRobotDynamicsObjectFactory *inObjFactory);

  /**
     \brief Initialization.
     \param inObjFactory factory necessary to build a CjrlDynamicHumanoidRobot.
  */

  ktStatus
    init(const ChppHumanoidRobotWkPtr& inWeakPtr, const std::string& inName);

  /**
     \brief Initialization with shared pointer.
  */

  ktStatus init(const ChppHumanoidRobotWkPtr& inWeakPtr,
		const ChppHumanoidRobotShPtr& inDevice);

private:

  /**
     \brief Store weak pointer to object.
  */
  ChppHumanoidRobotWkPtr attWeakPtr;

};

std::ostream& operator<<(std::ostream& os, ChppHumanoidRobot& inRobot);
#endif
