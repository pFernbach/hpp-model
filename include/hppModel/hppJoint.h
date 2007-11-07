/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINT_H
#define HPPJOINT_H

#include "KineoModel/kppJointComponent.h"
#include "hppModel/hppDevice.h"

KIT_PREDEF_CLASS(ChppJoint)
KIT_PREDEF_CLASS(ChppBody)
/**
   \brief Joint of a ChppDevice
*/

class ChppJoint {
public:
  /**
     \brief Constructor by geometric and dynamic joints
     \param inKppJoint Geometric part of the joint
     \param inJrlJoint Dynamic part of the joint
     \param inDeviceWeakPtr Weak pointer to the device that creates the joint.
  */
  ChppJoint(const CkppJointComponentShPtr& inKppJoint, CjrlJoint* inJrlJoint,
	    const ChppDeviceWkPtr& inDeviceWeakPtr) :
    attJrlJoint(inJrlJoint), attKppJoint(inKppJoint), attDevice(inDeviceWeakPtr) {};

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
  bool addChildJoint(ChppJoint* inJoint);
  
  /**
     @}
  */

  /**
     \name Bounds of the degrees of freedom
     @{
  */
  /**
     \brief Determine whether the degree of freedom of the joint is bounded

     \param inDofRank Rank of the degree of freedom that is bounded
     \param inBounded Whether the degree of freedom is bounded
  */
  inline void isBounded(unsigned int inDofRank, bool inBounded) {
    attKppJoint->kwsJoint()->dof(inDofRank)->isBounded(inBounded);
  };

  /**
     \brief Return whether the degree of freedom of the joint is bounded

     \param inDofRank Rank of the degree of freedom that is bounded
  */
  inline bool isBounded(unsigned int inDofRank)
  {
    return attKppJoint->kwsJoint()->dof(inDofRank)->isBounded();
  };

  /**
     \brief Get the lower bound of a given degree of freedom of the joint.
     
     \param inDofRank Id of the dof in the joint
  */
  inline double lowerBound(unsigned int inDofRank) const
  {
    return attKppJoint->kwsJoint()->dof(inDofRank)->vmin();
  };
  
  /**
     \brief Get the upper bound of a given degree of freedom of the joint.
     
     \param inDofRank Id of the dof in the joint
  */
  inline double upperBound(unsigned int inDofRank) const
  {
    return attKppJoint->kwsJoint()->dof(inDofRank)->vmax();
  };
  
  /**
     \brief Set the lower bound of a given degree of freedom of the joint.
     
     \param inDofRank Id of the dof in the joint
     \param inLowerBound lower bound
  */
  inline bool lowerBound(unsigned int inDofRank, double inLowerBound) 
  {
    // KPP side
    if (attKppJoint->kwsJoint()->dof(inDofRank)->vmin(inLowerBound)==false) {
      return false;
    }
    // CjrlJoint side
    attJrlJoint->lowerBound(inDofRank, inLowerBound);
    return true;
  };
  
  /**
     \brief Set the upper bound of a given degree of freedom of the joint.
     
     \param inDofRank Id of the dof in the joint
     \param inUpperBound Upper bound.
  */
  inline bool upperBound(unsigned int inDofRank, double inUpperBound)
  {
    // KPP side
    if (attKppJoint->kwsJoint()->dof(inDofRank)->vmax(inUpperBound)==false) {
      return false;
    }
    // CjrlJoint side
    attJrlJoint->upperBound(inDofRank, inUpperBound);
    return true;
  };
  
  /**
     \brief Set the bounds of the degrees of freedom of the joint

     \param inDofRank Rank of the degree of freedom that is bounded
     \param inLowerBound lower bound of this degree of freedom
     \param inUpperBound upper bound of this degree of freedom
  */
  inline bool bounds(unsigned int inDofRank, const double& inLowerBound, const double& inUpperBound) 
  {
    // KPP side
    if (attKppJoint->kwsJoint()->dof(inDofRank)->bounds(inLowerBound, inUpperBound)==false) {
      return false;
    }
    // CjrlJoint side
    attJrlJoint->lowerBound(inDofRank, inLowerBound);
    attJrlJoint->upperBound(inDofRank, inUpperBound);
    return true;
  };

  
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
     \name Conversion between KineoWorks and Matrix Abstraction Layer homogeneous Matrices
     @{
  */
  /**
     \brief Conversion from KineoWorks to Matrix Abstraction Layer
  */
  static matrix4d abstractMatrixFromCkitMat4(const CkitMat4& inMatrix);

  /**
     \brief Conversion from Matrix Abstraction Layer to KineoWorks
  */
  static CkitMat4 CkitMat4MatrixFromAbstract(const matrix4d& inMatrix);

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

  /**
     \brief Device for which this joint has been created for
     A joint is created by a device. It then only can be inserted in the kinematic chain of this device.
  */
  ChppDeviceWkPtr attDevice;

};


#endif
