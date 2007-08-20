/*
  Copyright 2007 LAAS-CNRS

  Author: Florent Lamiraux
*/

#include "hppModel/hppJoint.h"


ChppJointShPtr ChppJoint::parentJointHpp() const
{
  return KIT_DYNAMIC_PTR_CAST(ChppJoint, CkwsJoint::parentJoint());
}

bool ChppJoint::addChildJointHpp(const ChppJointShPtr& inJoint)
{
  if (CkwsJoint::addJoint(inJoint) != KD_OK) {
    return false;
  }
  if (!CimplJoint::addChildJoint(*inJoint)) {
    CkwsJoint::removeJoint(inJoint);
    return false;
  }
  return true;
}

ChppJointShPtr ChppJoint::childJointHpp(unsigned int inRank) const
{
  return KIT_DYNAMIC_PTR_CAST(ChppJoint, CkwsJoint::childJoint(inRank));
}
