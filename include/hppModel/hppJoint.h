/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINT_H
#define HPPJOINT_H

#include "KineoModel/kppJointComponent.h"
#include "hppModel/hppImplRobotDynamics.h"

KIT_PREDEF_CLASS(ChppJoint)
KIT_PREDEF_CLASS(ChppBody)
KIT_PREDEF_CLASS(ChppDevice)
/**
   \brief Joint of a ChppDevice
*/

class ChppJoint {
public:
  virtual ~ChppJoint() {};

  /**
     \name Access to geometric and dynamic parts of the joint.
     @{
  */
  /**
     \brief Access to geometric part of the joint.
  */
  CkppJointComponentShPtr kppJoint() { return attKppJoint;};

  /**
     \brief Access to dynamic part of the joint.
  */
  CjrlJoint* jrlJoint() { return attJrlJoint; };
  
  /**
     @}
  */

  /**
     \name Kinematic chain
     @{
  */

  /**
     \brief Get the parent joint
  */
  ChppJoint* parentJoint();
  
  /**
     \brief Get the child joint at a given rank.
  */
  ChppJoint* childJoint(unsigned int inRank);

  /**
     \brief Add a child to the joint
  */
  bool addChildJoint(ChppJoint& inJoint);
  
  /**
     @}
  */

  /**
     \name Attached body
     @{
  */
  /**
     \brief Attach a body to the joint
  */
  bool setAttachedBody(const ChppBodyShPtr& inBody);

  /**
     \brief Get the body attached to the joint
  */
  ChppBodyShPtr attachedBody();

  /**
     @}
  */

  /**
     \name Getting the device
     @{
  */

  /**
     \brief Get ChppDevice owning this joint
  */
  ChppDeviceShPtr hppDevice();

  /**
     @}
  */

  /**
     \name Creation of concrete joints
     @{
  */
  /**
     \brief Create a Free-flyer joint
  */
  static ChppJoint* createFreeFlyer(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     \brief Create a Free-flyer joint
  */
  static ChppJoint* createRotation(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     \brief Create a Free-flyer joint
  */
  static ChppJoint* createTranslation(std::string inName, const CkitMat4& inInitialPosition);
  
  /**
     @}
  */

protected:
  /**
     \brief Pointer to dynamic part of the joint
  */
  CjrlJoint* attJrlJoint;
  
  /**
     \brief Shared pointer to geometric part of joint
  */
  CkppJointComponentShPtr attKppJoint;

private:
  /**
     \brief template creation of joints to avoid duplicating code
  */
  template <class CkppJnt, class CjrlJnt> 
  static ChppJoint* createJoint(std::string inName, const CkitMat4& inInitialPosition);


};


#endif
