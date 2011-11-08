///
/// Copyright (c) 2011 CNRS
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

#ifndef HPP_MODEL_DEVICE_HH
#define HPP_MODEL_DEVICE_HH

#include <iostream>
#include <map>
#include <vector>

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "hpp/model/robot-dynamics-impl.hh"
#include "hpp/model/fwd.hh"

namespace hpp {
  namespace model {

/// \brief Robot with geometric and dynamic model

/// The dynamic model is implemented through robot dynamics
/// abstract interface, by impl::DynamicRobot which is an
/// implementation of CjrlDynamicRobot, while the geometric model
/// is implemented by CkppDeviceComponent (See KPP-SDK
/// documentation).

/// The creation of the device is done by Device::create(const
/// std::string name).  This function returns a shared pointer
/// to the newly created object.  \sa Smart pointers
/// documentation:
/// http://www.boost.org/libs/smart_ptr/smart_ptr.htm

/// Due to the composite nature of Device, configuration
/// vectors might differ between the geometric and the dynamic
/// parts. For this reason, two functions
/// Device::hppSetCurrentConfig and two functions
/// Device::hppGetCurrentConfig are are implemented.

class Device : public virtual impl::DynamicRobot,
		   public CkppDeviceComponent
{
public:
  /// \brief Specify which part of the device is concerned
  typedef enum EwhichPart {
	GEOMETRIC,
	DYNAMIC,
	BOTH
  } EwhichPart;

  static impl::ObjectFactory objectFactory_;

  /// \name Construction, copy and destruction
  /// @{
  virtual ~Device();

  /// \brief Clone as a CkwsDevice
  CkwsDeviceShPtr clone() const;

  /// \brief Clone as a CkppComponent
  CkppComponentShPtr cloneComponent() const;

  /// \brief Whether component is clonable.
  /// \return true
  bool isComponentClonable() const;

  ///
  /// @}
  ///

  ///
  /// \name Joints
  /// @{

  /// \brief Define the root joint
  void setRootJoint(JointShPtr joint);

  /// \brief Get the root joint
  JointShPtr getRootJoint();

  ///
  /// @}
  ///

  ///
  /// \name Configurations
  /// @{

  ///
  /// \brief Convert a KineoWorks config into a jrlDynamicRobot config

  /// \param kwsDofVector vector of degrees of freedom of CkwsConfig
  /// \retval outJrlDynamicsDofVector vector of degrees of freedom of jrlDynamicRobot config
  /// \pre outJrlDynamicsDofVector.size() == numberDof()

  /// \note KineoWorks configurations are represented as dof
  /// values instead of CkwsConfig since objects of this latter
  /// type are subject to the constraints of the device they
  /// belong to.
  /// \return true if success, false if error.
  bool kwsToJrlDynamicsDofValues(const std::vector<double>& kwsDofVector,
				 vectorN& outJrlDynamicsDofVector);

  /// \brief Convert a jrlDynamicRobot config into a KineoWorks config
  /// \param inJrlDynamicsDofVector vector of degrees of freedom of jrlDynamicRobot config
  /// \retval outKwsDofVector vector of degrees of freedom of CkwsConfig
  /// \note KineoWorks configurations are represented as dof
  /// values instead of CkwsConfig since objects of this latter
  /// type are subject to the constraints of the device they
  /// belong to.
  /// \return true if success, false if error.
  bool jrlDynamicsToKwsDofValues(const vectorN& inJrlDynamicsDofVector,
				 std::vector<double>& outKwsDofVector);

  /** \brief Conversion of rotation

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
  \f} */
  void RollPitchYawToYawPitchRoll(const double& inRx, const double& inRy,
				  const double& inRz,
				  double& outRx, double& outRy,
				  double& outRz);

  /** \brief Conversion of rotation
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
  \f} */
  void YawPitchRollToRollPitchYaw(const double& inRx, const double& inRy,
				  const double& inRz,
				  double& outRx, double& outRy,
				  double& outRz);

  /// \brief Put the robot in a given configuration

  /// \param config The configuration
  /// \param updateWhat Specify which part of the robot (geometric, dynamic or both) should be update)

  /// \return true if success, false otherwise.

  /// \note In CkwsConfig, the order of the joint
  /// degrees-of-freedom follow KineoWorks convention.  The
  /// configuration of the dynamic part (impl::DynamicRobot) is
  /// thus computed accordingly.
  bool hppSetCurrentConfig(const CkwsConfig& config,
			   EwhichPart updateWhat=BOTH);

  /// \brief Put the robot in a given configuration

  /// \param config The configuration
  /// \param updateWhat Specify which part of the robot (geometric, dynamic or both) should be update)
  
  /// \return true if success, false otherwise.
  
  /// CkppDeviceComponent extra-dofs are set to 0.
  
  /// \note In vectorN, the order of the joint degrees-of-freedom
  /// follow impl::DynamicRobot convention. The configuration of
  /// the geometric part (CkppDeviceComponent) is thus computed
  /// accordingly.
  bool hppSetCurrentConfig(const vectorN& config,
			   EwhichPart updateWhat=BOTH);

  ///
  /// @}
  ///

  ///
  /// \name Collision checking and distance computations
  /// @{
  ///
  /// \brief Set collision checking in such a way that given device is ignored by this one.
  /// \param device The device to be ignored.
  ktStatus ignoreDeviceForCollision (DeviceShPtr device) ;

  /// \brief Add obstacle to the list.
  /// \param object a new object.
  /// \param distanceComputation whether this object should be taken into
  /// account for distance computation for all bodies.
  ktStatus addObstacle(const CkcdObjectShPtr& object,
			   bool distanceComputation=false);

  ///
  /// @}
  ///

  ///
  /// \name Bounding box
  /// @{
  ///
  /// \brief Compute the bounding box of the robot in current configuration.
  ///
  ktStatus axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
				   double& xMax, double& yMax, double& zMax)
	const;

  ///
  /// @}
  ///

  /// \brief Called whenever a child component is inserted
  /// This function enables the object to update information provided
  /// through properties when joints are inserted to the robot.
  void componentDidInsertChild
  (const CkitNotificationConstShPtr& notification);

  /// \brief Called before a child component is inserted
  /// This function enables the object to update information provided
  /// through properties when joints are inserted to the robot.
  void componentWillInsertChild
  (const CkitNotificationConstShPtr& notification);

  /// \brief Initialize kinematic chain
  /// If dynamic part of joints have not been created, create them.
  /// In any case, call parent implementation.
  /// \note When reading a kxml file, the position of joints is not
  /// known at construction. For this reason, the dynamic part of each
  /// joint is created later by this function.
  virtual bool initialize ();

  /// \brief Creation of a new device
  /// \return a shared pointer to the new device
  /// \param name Name of the device (is passed to CkkpDeviceComponent)
  static DeviceShPtr create(std::string name);

  ///
  /// \brief Copy of a device
  /// \return A shared pointer to new device.
  /// \param device Device to be copied.
  static DeviceShPtr createCopy(const DeviceShPtr& device);

  /// \brief Access to the object factory for the dynamic part.
  static impl::ObjectFactory* objectFactory();

protected:
  /// \brief Constructor
  /// \param objFactory factory necessary to build a CjrlDynamicRobot.
  Device(CjrlRobotDynamicsObjectFactory *objFactory);

  Device();

  ///
  /// \brief Initialization.
  ///
  ktStatus init(const DeviceWkPtr& weakPtr, const std::string& name);

  ///
  /// \brief Initialization with shared pointer.
  ///
  ktStatus init(const DeviceWkPtr& weakPtr, const DeviceShPtr& device);

  /// \brief Insert child dynamic part in parent dynamic part.
  /// \input parent Parent joint,
  /// \input child Child joint.
  /// This method is called whenever a Joint object is inserted as child
  /// component of another Joint object.
  void insertDynamicPart(JointShPtr parent, JointShPtr child);

private:

  /// \brief Store weak pointer to object.
  DeviceWkPtr weakPtr_;

  void computeBodyBoundingBox(const CkwsKCDBodyShPtr& body, double& xMin,
				  double& yMin, double& zMin, double& xMax,
				  double& yMax, double& zMax) const;

  void ckcdObjectBoundingBox(const CkcdObjectShPtr& object, double& xMin,
				 double& yMin, double& zMin, double& xMax,
				 double& yMax, double& zMax) const;

  void initializeKinematicChain(JointShPtr joint);
}; // class Device
  } // namespace model
} // namespace hpp
std::ostream& operator<<(std::ostream& os, hpp::model::Device& inHppDevice);

#endif // HPP_MODEL_DEVICE_HH
