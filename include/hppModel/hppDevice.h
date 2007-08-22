/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPPDEVICE_H
#define HPPDEVICE_H

#include <map>

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "hppModel/hppJoint.h"

KIT_PREDEF_CLASS(ChppDevice);

/**
 \brief Robot with geometric and dynamic model 
 
 The dynamic model is implemented through robotDynamics abstract interface, while the geometric model is implemented by KineoWorks.

 The class is templated by an implementation of CjrlDynamicRobot. Thus, different implementations can be used without modification of the code.

 The creation of the device is done by ChppDevice::create(const std::string inName). This function returns a shared pointer to the newly created object.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/

class ChppDevice : public CkppDeviceComponent, public CimplDynamicRobot {
public:
  /**
     \name Construction, copy and destruction
     {@
  */
  virtual ~ChppDevice();

  /**
     \brief Creation of a new device
     \return a shared pointer to the new device
     \param inName Name of the device (is passed to CkkpDeviceComponent)
  */
  static ChppDeviceShPtr create(std::string inName);

  /**
     \brief Copy of a device
     \return A shared pointer to new device.
     \param inDevice Device to be copied.
  */
  static ChppDeviceShPtr createCopy(const ChppDeviceShPtr& inDevice);

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
     \name Joints
  */

  /**
     \brief Define the root joint
  */
  void setRootJoint(ChppJoint& inJoint);

  /**
     \brief Get the root joint
  */
  ChppJoint* getRootJoint();

  /**
     \brief Get ChppJoint containing a given CkppJointComponent
  */
  ChppJoint* kppToHppJoint(CkppJointComponentShPtr inKppJoint);

  /**
     \brief Register joint in device.
  */
  void registerJoint(ChppJoint& inHppJoint);

  /**
     @}
  */

  /**
     \name Collision checking and distance computations
     @{
  */
  /**
     \brief Set collision checking in such a way that given device is ignored by this one.
     \param inDevice The device to be ignored.

  */
  ktStatus ignoreDeviceForCollision (ChppDeviceShPtr inDevice) ;

  /**
   * \brief Add obstacle to the list.
   * \param inObject a new object.
   * \note Compute collision entities.
   */
  ktStatus addObstacle(const CkcdObjectShPtr& inObject);

  /**
     @}
  */

  /**
     \name Bounding box
     @{
  */
  /**
     \brief Compute the bounding box of the robot in current configuration.
  */
  ktStatus axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
				   double& xMax, double& yMax, double& zMax) const;

  /**
     @}
  */

protected:
  /**
     \brief Constructor
  */
  ChppDevice();

  /**
     \brief Initialization.
  */

  ktStatus init(const ChppDeviceWkPtr& inWeakPtr, const std::string& inName);

  /**
     \brief Initialization with shared pointer.
  */

  ktStatus init(const ChppDeviceWkPtr& inWeakPtr, const ChppDeviceShPtr& inDevice);

private:

  /**
     \brief Store weak pointer to object.
  */
  ChppDeviceWkPtr attWeakPtr;
  
  void computeBodyBoundingBox(const CkwsKCDBodyShPtr& body, double& xMin, double& yMin, 
			      double& zMin, double& xMax, double& yMax, double& zMax) const;

  void ckcdObjectBoundingBox(const CkcdObjectShPtr& object, double& xMin, double& yMin, 
			     double& zMin, double& xMax, double& yMax, double& zMax) const;

  /**
     \brief Map to retrieve the ChppJoint that contains a given CjrlJoint
  */
  std::map<CjrlJoint*, ChppJoint*> attJrlToHppJointMap;

  /**
     \brief Map to retrieve the ChppJoint by name.
  */
  std::map<CkppJointComponent*, ChppJoint*> attKppToHppJointMap;
};

#endif
