/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINTFREEFLYER_H
#define HPPJOINTFREEFLYER_H

#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "hppModel/hppJoint.h"

KIT_PREDEF_CLASS(ChppJointFreeFlyer)

/**
   \brief Free-flyer joint of a ChppDevice.

   Derives from
   \li CimplJointFreeFlyer: a free-flyer implementation of CjrlJoint,
   \li ChppJoint,
   \li CkwsJointFreeFlyer.
*/

class ChppJointFreeFlyer : public CimplJointFreeFlyer, public ChppJoint, public CkppFreeFlyerJointComponent {
public:
  ~ChppJointFreeFlyer();

  /**
     \brief Return a shared pointer to a new free-flyer joint.
     \param inName Name of the joint
     \param inPosMat Position of the joint
  */
  static ChppJointFreeFlyerShPtr create(std::string inName, const CkitMat4 &inPosMat=kitDefaultMat4);

  /**
     \brief Return a shared pointer to a copy of the input joint
     \param inJoint Joint to copy.
  */
  static ChppJointFreeFlyerShPtr createCopy(const ChppJointFreeFlyerShPtr& inJoint);

  /**
     \brief Clone as a CkwsJoint
  */
  virtual CkwsJointShPtr clone() const;

protected:

  /**
     \brief name Reimplementation of CkwsJoint methods
     @{
  */
  virtual CkwsJointDofShPtr makeDof(unsigned int iDof) const;
  virtual void maxSpeed(const double* i_maxDofSpeeds, double& o_vMax, double& o_wMax) const;
  virtual void computeEffect(const double* i_dofValues, CkitMat4& o_effectMatrix) const;

  /**
     @}
  */
  /**
     \brief Initialization
     \param inWeakPtr Weak pointer on joint to initialize
     \param inName Name of the joint
  */
  ktStatus init(const ChppJointFreeFlyerWkPtr& inWeakPtr, std::string inName);

  /**
     \brief Initialization
  */
  ktStatus init(const ChppJointFreeFlyerWkPtr& inWeakPtr, const ChppJointFreeFlyerShPtr& inJoint);

  /**
     \brief Constructor by position
  */
  ChppJointFreeFlyer(const CkitMat4 &inPosMat);

  /**
     \brief Copy constructor 
  */
  ChppJointFreeFlyer(ChppJointFreeFlyer& inJoint);

private:
  /**
     \brief Weak pointer to itself
  */
  ChppJointFreeFlyerWkPtr attWeakPtr;
};

#endif
