/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPPSPECIFICHUMANOIDROBOT_H
#define HPPSPECIFICHUMANOIDROBOT_H

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "robotDynamics/jrlHumanoidDynamicRobot.h"
#include "hppModel/hppHumanoidRobot.h"

template <class HDR = CimplHumanoidDynamicRobot>
  class ChppSpecificHumanoidRobot;

#define ChppSpecificHumanoidRobotShPtr \
  boost::shared_ptr< ChppSpecificHumanoidRobot<HDR> >
#define ChppSpecificHumanoidRobotWkPtr \
  boost::weak_ptr< ChppSpecificHumanoidRobot<HDR> >

/**
   \brief Specific implementation of humanoid robot with geometric and dynamic model .

   This template class enables a developer to define a composite humanoid robot class based on an optimized implementation of CimplHumanoidDynamicRobot. To do so,
   \li derive CimplHumanoidDynamicRobot into CoptHumanoidDynamicRobot and overload the methods you want to optimize,
   \li define your own ChppSpecificHumanoidRobot class by instanciating the template with your implementation:
   \code
   typedef ChppSpecificHumanoidRobot<CoptHumanoidDynamicRobot> CyourOptHppHumanoidRobot;
   \endcode

   \image html classDiagramHumanoid.png "Inheritance diagram of a composite humanoid robot class based on an optimized implementation of the humanoid robot dynamic model."
*/

template <class HDR> class ChppSpecificHumanoidRobot :
public ChppHumanoidRobot, public HDR
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
  static ChppSpecificHumanoidRobotShPtr create(std::string inName);

  /**
     \brief Copy of a device
     \return A shared pointer to new device.
     \param inDevice Device to be copied.
  */
  static ChppSpecificHumanoidRobotShPtr
    createCopy(const ChppSpecificHumanoidRobotShPtr& inDevice);

protected:
  /**
     \brief Initialization.
  */

  ktStatus init(const ChppSpecificHumanoidRobotWkPtr& inWeakPtr,
		const std::string& inName);

  /**
     \brief Initialization with shared pointer.
  */

  ktStatus init(const ChppSpecificHumanoidRobotWkPtr& inWeakPtr,
		const ChppSpecificHumanoidRobotShPtr& inDevice);

private:

  /**
     \brief Store weak pointer to object.
  */
  ChppSpecificHumanoidRobotWkPtr attWeakPtr;

};


// ==========================================================================

template <class HDR> ChppSpecificHumanoidRobotShPtr
ChppSpecificHumanoidRobot<HDR>::create(std::string inName)
{
  ChppSpecificHumanoidRobot<HDR> *hppDevice =
    new ChppSpecificHumanoidRobot<HDR>;
  ChppSpecificHumanoidRobotShPtr hppDeviceShPtr(hppDevice);

  if (hppDevice->init(hppDeviceShPtr, inName) != KD_OK) {
    hppDeviceShPtr.reset();
  }
  return hppDeviceShPtr;
}

// ==========================================================================

template <class HDR> ChppSpecificHumanoidRobotShPtr
ChppSpecificHumanoidRobot<HDR>::createCopy
(const ChppSpecificHumanoidRobotShPtr& inDevice)
{
  ChppSpecificHumanoidRobot<HDR>* ptr =
    new ChppSpecificHumanoidRobot<HDR>(*inDevice);
  ChppSpecificHumanoidRobotShPtr deviceShPtr(ptr);

  if(KD_OK != ptr->init(deviceShPtr, inDevice))	{
    deviceShPtr.reset();
  }

  return deviceShPtr;
}

// ==========================================================================

template <class HDR> CkwsDeviceShPtr
ChppSpecificHumanoidRobot<HDR>::clone() const
{
  return ChppSpecificHumanoidRobot<HDR>::createCopy(attWeakPtr.lock());
}

// ==========================================================================

template <class HDR> CkppComponentShPtr
ChppSpecificHumanoidRobot<HDR>::cloneComponent() const
{
  return ChppSpecificHumanoidRobot<HDR>::createCopy(attWeakPtr.lock());
}

// ==========================================================================

template <class HDR> bool
ChppSpecificHumanoidRobot<HDR>::isComponentClonable() const
{
  return true;
}

// ==========================================================================

template <class HDR> ktStatus
ChppSpecificHumanoidRobot<HDR>::init
(const ChppSpecificHumanoidRobotWkPtr& inDevWkPtr, const std::string &inName)
{
  ktStatus success = ChppDevice::init(inDevWkPtr, inName);

  if(KD_OK == success) {
    attWeakPtr = inDevWkPtr;
  }
  return success;
}

// ==========================================================================

template <class HDR> ktStatus
ChppSpecificHumanoidRobot<HDR>::init
(const ChppSpecificHumanoidRobotWkPtr& inWeakPtr,
 const ChppSpecificHumanoidRobotShPtr& inDevice)
{
  ktStatus  success = ChppDevice::init(inWeakPtr, inDevice);

  if(KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}

#endif
