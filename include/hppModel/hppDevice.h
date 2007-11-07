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

#include "hppModel/hppImplRobotDynamics.h"

class ChppJoint;
KIT_PREDEF_CLASS(ChppDevice);

/**
   \brief Robot with geometric and dynamic model 
 
   The dynamic model is implemented through robotDynamics abstract interface, by CimplDynamicRobot 
   which is an implementation of CjrlDynamicRobot, while the geometric model is implemented by 
   CkppDeviceComponent (See KPP-SDK documentation).
   
   The creation of the device is done by ChppDevice::create(const std::string inName). 
   This function returns a shared pointer to the newly created object.
   \sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
   
   Due to the composite nature of ChppDevice, configuration vectors might differ between the geometric and 
   the dynamic parts. For this reason, two functions ChppDevice::hppSetCurrentConfig and two functions 
   ChppDevice::hppGetCurrentConfig are are implemented.
*/

class ChppDevice : public CkppDeviceComponent, public virtual CimplDynamicRobot {
public:
  /**
     \brief Specify which part of the device is concerned
  */
  typedef enum EwhichPart {
    GEOMETRIC,
    DYNAMIC,
    BOTH
  } EwhichPart;

  /**
     \name Construction, copy and destruction
     @{
  */
  virtual ~ChppDevice();

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
     @{
  */

  /**
     \brief Define the root joint
  */
  void setRootJoint(ChppJoint* inJoint);

  /**
     \brief Get the root joint
  */
  ChppJoint* getRootJoint();

  /**
     \brief Get ChppJoint containing a given CkppJointComponent
  */
  ChppJoint* kppToHppJoint(CkppJointComponentShPtr inKppJoint);

  /**
     \brief Create a Free-flyer joint
  */
  ChppJoint* createFreeFlyer(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     \brief Create a Free-flyer joint
  */
  ChppJoint* createRotation(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     \brief Create a Free-flyer joint
  */
  ChppJoint* createTranslation(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     @}
  */

  /**
     \name Configurations
     @{
  */

  /**
     \brief Put the robot in a given configuration

     \param inConfig The configuration
     \param inUpdateWhat Specify which part of the robot (geometric, dynamic or both) should be update)

     \return true if success, false otherwise.

     \note In CkwsConfig, the order of the joint degrees-of-freedom follow KineoWorks convention.
     The configuration of the dynamic part (CimplDynamicRobot) is thus computed accordingly.
  */
  bool hppSetCurrentConfig(const CkwsConfig& inConfig, EwhichPart inUpdateWhat=BOTH);

  /**
     \brief Put the robot in a given configuration

     \param inConfig The configuration
     \param inUpdateWhat Specify which part of the robot (geometric, dynamic or both) should be update)

     \return true if success, false otherwise.
     
     CkppDeviceComponent extra-dofs are set to 0.

     \note In vectorN, the order of  the joint degrees-of-freedom follow CimplDynamicRobot convention.
     The configuration of the geometric part (CkppDeviceComponent) is thus computed accordingly.
  */
  bool hppSetCurrentConfig(const vectorN& inConfig, EwhichPart inUpdateWhat=BOTH);

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

  /**
     \brief Register joint in device.
  */
  void registerJoint(ChppJoint* inHppJoint);

  /**
     \brief template creation of joints to avoid duplicating code
  */
  template <class CkppJnt, class CjrlJnt> ChppJoint* createJoint(std::string inName, const CkitMat4& inInitialPosition);

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
     \brief Conversion of rotation 

     Convert 3D-rotation from standard (Roll, Pitch, Yaw) coordinates to Kineo (Yaw, Pitch, Roll) coordinates
     \f{eqnarray*}
     R_{rpy}(inRx,inRy,inRz)&=&
     \left(\begin{array}{ccc}
     \cos(inRy)\ \cos(inRz) & \sin(inRx)\ \sin(inRy)\ \cos(inRz) - \cos(inRx)\ \sin(inRz) & \cos(inRx)\ \sin(inRy)\ \cos(inRz) + \sin(inRx)\ \sin(inRz) \\
     \cos(inRy)\ \sin(inRz) & \sin(inRx)\ \sin(inRy)\ \sin(inRz) + \cos(inRx)\ \cos(inRz) & \cos(inRx)\ \sin(inRy)\ \sin(inRz) - \sin(inRx)\ \cos(inRz) \\
     -\sin(inRy) & \sin(inRx)\ \cos(inRy) & \cos(inRx)\ \cos(inRy) 
     \end{array}\right) \\
     \\
     &=&
     \left( \begin {array}{ccc} \cos \left( {\it outRz} \right) \cos \left( {\it outRy} \right) &-\sin \left( {\it outRz} \right) \cos
     \left( {\it outRy} \right) &\sin \left( {\it outRy} \right) \\\noalign{\medskip}\cos \left( {\it outRz} \right) \sin \left( {\it outRy}
     \right) \sin \left( {\it outRx} \right) +\sin \left( {\it outRz} \right) \cos \left( {\it outRx} \right) &\cos \left( {\it outRz}
     \right) \cos \left( {\it outRx} \right) -\sin \left( {\it outRz} \right) \sin \left( {\it outRy} \right) \sin \left( {\it outRx} \right)
     &-\cos \left( {\it outRy} \right) \sin \left( {\it outRx} \right) \\\noalign{\medskip}\sin \left( {\it outRz} \right) \sin \left( {\it
     outRx} \right) -\cos \left( {\it outRz} \right) \sin \left( {\it outRy} \right) \cos \left( {\it outRx} \right) &\sin \left( {\it outRz}
     \right) \sin \left( {\it outRy} \right) \cos \left( {\it outRx} \right) +\cos \left( {\it outRz} \right) \sin \left( {\it outRx} \right)
     &\cos \left( {\it outRy} \right) \cos \left( {\it outRx} \right) \end {array} \right)\\
     \\
     &=&
     R_{ypr}(outRx, outRy, outRz)\\
     \\
     &&-\pi < outRx \leq \pi\\
     &&-\pi/2 <outRy \leq \pi/2 \\
     &&-\pi < outRz \leq \pi\\
     \f}



  */
  void RollPitchYawToYawPitchRoll(const double& inRx, const double& inRy, const double& inRz,
				  double& outRx, double& outRy, double& outRz);

  /**
     \brief Conversion of rotation 

     Convert 3D-rotation from Kineo (Yaw, Pitch, Roll) coordinates to standard (Roll, Pitch, Yaw) coordinates
     \f{eqnarray*}
     R_{ypr}(inRx,inRy,inRz)&=&
     \left( \begin {array}{ccc} \cos \left( {\it inRz} \right) \cos \left( {\it inRy} \right) &-\sin \left( {\it inRz} \right) \cos
     \left( {\it inRy} \right) &\sin \left( {\it inRy} \right) \\\noalign{\medskip}\cos \left( {\it inRz} \right) \sin \left( {\it inRy}
     \right) \sin \left( {\it inRx} \right) +\sin \left( {\it inRz} \right) \cos \left( {\it inRx} \right) &\cos \left( {\it inRz}
     \right) \cos \left( {\it inRx} \right) -\sin \left( {\it inRz} \right) \sin \left( {\it inRy} \right) \sin \left( {\it inRx} \right)
     &-\cos \left( {\it inRy} \right) \sin \left( {\it inRx} \right) \\\noalign{\medskip}\sin \left( {\it inRz} \right) \sin \left( {\it
     inRx} \right) -\cos \left( {\it inRz} \right) \sin \left( {\it inRy} \right) \cos \left( {\it inRx} \right) &\sin \left( {\it inRz}
     \right) \sin \left( {\it inRy} \right) \cos \left( {\it inRx} \right) +\cos \left( {\it inRz} \right) \sin \left( {\it inRx} \right)
     &\cos \left( {\it inRy} \right) \cos \left( {\it inRx} \right) \end {array} \right)\\
     \\
     &=&
     \left(\begin{array}{ccc}
     \cos(outRy)\ \cos(outRz) & \sin(outRx)\ \sin(outRy)\ \cos(outRz) - \cos(outRx)\ \sin(outRz) & \cos(outRx)\ \sin(outRy)\ \cos(outRz) + \sin(outRx)\ \sin(outRz) \\
     \cos(outRy)\ \sin(outRz) & \sin(outRx)\ \sin(outRy)\ \sin(outRz) + \cos(outRx)\ \cos(outRz) & \cos(outRx)\ \sin(outRy)\ \sin(outRz) - \sin(outRx)\ \cos(outRz) \\
     -\sin(outRy) & \sin(outRx)\ \cos(outRy) & \cos(outRx)\ \cos(outRy) 
     \end{array}\right) \\
     \\
     &=&
     R_{rpy}(outRx, outRy, outRz)\\
     \\
     &&-\pi < outRx \leq \pi\\
     &&-\pi/2 <outRy \leq \pi/2 \\
     &&-\pi < outRz \leq \pi\\
     \f}



  */
  void YawPitchRollToRollPitchYaw(const double& inRx, const double& inRy, const double& inRz,
				  double& outRx, double& outRy, double& outRz);

  /**
     \brief Map to retrieve the ChppJoint that contains a given CjrlJoint
  */
  std::map<CjrlJoint*, ChppJoint*> attJrlToHppJointMap;

  /**
     \brief Map to retrieve the ChppJoint by name.
  */
  std::map<CkppJointComponentShPtr, ChppJoint*> attKppToHppJointMap;
};

#endif
