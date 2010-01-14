/*
  Copyright 2007 LAAS-CNRS

  Author: Florent Lamiraux
*/

#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoModel/kppRotationJointComponent.h"
#include "KineoModel/kppTranslationJointComponent.h"

#include "kwsIO/kwsioMat.h"
#include "KineoUtility/kitDefine.h"

#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "hppModel/hppDevice.h"
#include "hppModel/hppBody.h"
#include "hppModel/hppJoint.h"

matrix4d ChppJoint::abstractMatrixFromCkitMat4(const CkitMat4& inMatrix)
{
  MAL_S4x4_MATRIX(abstractMatrix, double);
  for (unsigned int iRow=0; iRow<4; iRow++) {
    for (unsigned int iCol=0; iCol<4; iCol++) {
      MAL_S4x4_MATRIX_ACCESS_I_J(abstractMatrix, iRow, iCol) =
	inMatrix(iRow, iCol);
    }
  }
  return abstractMatrix;
}

CkitMat4 ChppJoint::CkitMat4MatrixFromAbstract(const matrix4d& inMatrix)
{
  CkitMat4 kitMat4;
  for (unsigned int iRow=0; iRow<4; iRow++) {
    for (unsigned int iCol=0; iCol<4; iCol++) {
      kitMat4(iRow, iCol) = MAL_S4x4_MATRIX_ACCESS_I_J(inMatrix, iRow, iCol);
    }
  }
  return kitMat4;
}

// ==========================================================================

ChppJoint* ChppJoint::parentJoint()
{
  /*
    Get the device of the joint if any
  */
  if (ChppDeviceShPtr device = hppDevice()) {
    CkppComponentShPtr parentComponent = attKppJoint->parent();
    CkppJointComponentShPtr parentJoint;
    if (parentJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent,
					   parentComponent)) {
      return device->kppToHppJoint(parentJoint);
    }
  }
  return NULL;
}


// ==========================================================================

ChppJoint* ChppJoint::childJoint(unsigned int inRank)
{
  /*
    Get the device of the joint if any
  */
  if (ChppDeviceShPtr device = hppDevice()) {
    CkppJointComponentShPtr childComponent =
      attKppJoint->childJointComponent(inRank);
    return device->kppToHppJoint(childComponent);
  }

  return NULL;
}

// ==========================================================================

bool ChppJoint::addChildJoint(ChppJoint* inJoint)
{
  /*
    Check that parent and child have been created by the same device
  */
  if (hppDevice() == inJoint->hppDevice()) {
    if (attKppJoint->addChildJointComponent(inJoint->attKppJoint) != KD_OK) {
      return false;
    }
    if (!attJrlJoint->addChildJoint(*(inJoint->attJrlJoint))) {
      attKppJoint->removeChildJointComponent(inJoint->attKppJoint);
      return false;
    }
    return true;
  }
  std::cerr <<
    "ChppJoint::addChildJoint: joints have not been created by the same device"
	    << std::endl;
  return false;
}

// ==========================================================================

unsigned int ChppJoint::countChildJoints()
{
  if (attKppJoint) {
    return attKppJoint->countChildJointComponents();
  }
  return 0;
}

// ==========================================================================

ChppDeviceShPtr ChppJoint::hppDevice()
{
  return attDevice.lock();
}


// ==========================================================================

bool ChppJoint::setAttachedBody(const ChppBodyShPtr& inBody)
{
  /*
    Attach the body to the geometric part of the joint
  */
  kppJoint()->kwsJoint()->setAttachedBody(inBody);
  /*
    Attach the body to the dynamic part of the joint
  */
  jrlJoint()->setLinkedBody(*(inBody.get()));

  /*
    Store pointer to the joint in body
  */
  inBody->hppJoint(this);
  return true;
}

// ==========================================================================

ChppBodyShPtr ChppJoint::attachedBody()
{
  return KIT_DYNAMIC_PTR_CAST
    (ChppBody, kppJoint()->kwsJoint()->attachedBody());
}

std::ostream& operator<<(std::ostream& os, ChppJoint& inHppJoint)
{
  os << "Joint: " << inHppJoint.kppJoint()->name() << std::endl;
  os << "Rank in configuration dynamic part: " 
     << inHppJoint.jrlJoint()->rankInConfiguration()
     << std::endl;
  os << "Current transformation dynamic part:" << std::endl;
  os << inHppJoint.jrlJoint()->currentTransformation() << std:: endl;
  os << std::endl;
  os << "Current transformation geometric part:" << std::endl;
  os << inHppJoint.kppJoint()->kwsJoint()->currentPosition() << std:: endl;

  ChppBodyShPtr hppBody = inHppJoint.attachedBody();
  if (hppBody) {
    os << "Attached body:" << std::endl;
    os << "Mass of the attached body: " << hppBody->mass() << std::endl;
    os << "Local center of mass:" << hppBody->localCenterOfMass() << std::endl;
    os << "Inertia matrix:" << std::endl;
    os << hppBody->inertiaMatrix() << std::endl;
  } else {
    os << "No attached body" << std::endl;
  }

  for (unsigned int iChild=0; iChild < inHppJoint.countChildJoints();
       iChild++) {
    os << *(inHppJoint.childJoint(iChild)) << std::endl;
    os <<std::endl;
  }

  return os;
}
