/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINT_H
#define HPPJOINT_H

#include "KineoWorks2/kwsJoint.h"
#include "hppModel/hppImplRobotDynamics.h"

KIT_PREDEF_CLASS(ChppJoint)

/**
   \brief Joint of a ChppDevice
*/

class ChppJoint : public CimplJoint, virtual public CkwsJoint {
public:
  virtual ~ChppJoint() {};

    /**
       \brief Get the parent joint
    */
    ChppJointShPtr parentJointHpp() const;

    /**
       \brief Add a child to the joint
    */
    bool addChildJointHpp(const ChppJointShPtr& inJoint);

    /**
       \brief Get the child joint at a given rank.
    */
    ChppJointShPtr childJointHpp(unsigned int inRank) const;

#if 0
    /**
       \brief Clone as a CkwsJoint
    */
    virtual CkwsJointShPtr clone() const = 0;
#endif
protected:
#if 0
  virtual CkwsJointDofShPtr makeDof(unsigned int iDof) const = 0;
  virtual void maxSpeed(const double* i_maxDofSpeeds, double& o_vMax, double& o_wMax) const = 0;
  virtual void computeEffect(const double* i_dofValues, CkitMat4& o_effectMatrix) const = 0;
#endif
};


#endif
