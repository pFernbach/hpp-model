/*
  Copyright 2007 LAAS-CNRS 
  Author: Florent Lamiraux
*/

#include "hppModel/hppJointFreeFlyer.h"

static matrix4d abstractMatrixFromCkitMat4(const CkitMat4& matrix)
{
  MAL_S4x4_MATRIX(abstractMatrix, double);
  for (unsigned int iRow=0; iRow<3; iRow++) {
    for (unsigned int iCol=0; iCol<3; iCol++) {
      MAL_S4x4_MATRIX_ACCESS_I_J(abstractMatrix, iRow, iCol) = matrix(iRow, iCol);
    }
  }
  return abstractMatrix;
}

ChppJointFreeFlyerShPtr ChppJointFreeFlyer::create(std::string inName, const CkitMat4 &inPosMat)
{
  ChppJointFreeFlyer* jointPtr = new ChppJointFreeFlyer(inPosMat);
  ChppJointFreeFlyerShPtr jointShPtr(jointPtr);

  if (jointPtr->init(jointShPtr, inName) != KD_OK) {
    jointShPtr.reset();
  }
  return jointShPtr;
}

ChppJointFreeFlyerShPtr ChppJointFreeFlyer::createCopy(const ChppJointFreeFlyerShPtr& inJoint)
{
  ChppJointFreeFlyer* jointPtr = new ChppJointFreeFlyer(*inJoint);
  ChppJointFreeFlyerShPtr jointShPtr(jointPtr);

  if (jointPtr->init(jointShPtr, inJoint) != KD_OK) {
    jointShPtr.reset();
  }
  return jointShPtr;
}


ktStatus ChppJointFreeFlyer::init(const ChppJointFreeFlyerWkPtr& inWeakPtr, std::string inName)
{
  ktStatus status = CkppFreeFlyerJointComponent::init(inWeakPtr, inName);
  if (status == KD_OK) {
    attWeakPtr = inWeakPtr;
  }
  return status;
}

ktStatus ChppJointFreeFlyer::init(const ChppJointFreeFlyerWkPtr& inWeakPtr, const ChppJointFreeFlyerShPtr& inJoint)
{
  ktStatus status = CkppFreeFlyerJointComponent::init(inWeakPtr, inJoint);
  if (status == KD_OK) {
    attWeakPtr = inWeakPtr;
  }
  return status;
}


ChppJointFreeFlyer::ChppJointFreeFlyer(const CkitMat4 &inPosMat) : 
  CimplJointFreeFlyer(abstractMatrixFromCkitMat4(inPosMat)), CkppFreeFlyerJointComponent()
{
  this->CkwsJointFreeFlyer::setCurrentPosition(inPosMat);
}

ChppJointFreeFlyer::ChppJointFreeFlyer(ChppJointFreeFlyer& inJoint) :
  CimplJointFreeFlyer(inJoint.CimplJointFreeFlyer::initialPosition()),
  CkppFreeFlyerJointComponent(inJoint)
{
}

CkwsJointShPtr ChppJointFreeFlyer::clone() const
{
  return CkwsJointFreeFlyer::clone();
}

CkwsJointDofShPtr ChppJointFreeFlyer::makeDof(unsigned int iDof) const
{
  return CkwsJointFreeFlyer::makeDof(iDof);
}

void ChppJointFreeFlyer::maxSpeed(const double* i_maxDofSpeeds, double& o_vMax, double& o_wMax) const
{
  return CkwsJointFreeFlyer::maxSpeed(i_maxDofSpeeds, o_vMax, o_wMax);
}

void ChppJointFreeFlyer::computeEffect(const double* i_dofValues, CkitMat4& o_effectMatrix) const
{
  return CkwsJointFreeFlyer::computeEffect(i_dofValues, o_effectMatrix);
}

