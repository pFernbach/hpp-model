/*
  Copyright 2007 LAAS-CNRS

  Author: Florent Lamiraux
*/

#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoModel/kppRotationJointComponent.h"
#include "KineoModel/kppTranslationJointComponent.h"

#include "hppModel/hppDevice.h"
#include "hppModel/hppBody.h"

static matrix4d abstractMatrixFromCkitMat4(const CkitMat4& inMatrix)
{
  MAL_S4x4_MATRIX(abstractMatrix, double);
  for (unsigned int iRow=0; iRow<3; iRow++) {
    for (unsigned int iCol=0; iCol<3; iCol++) {
      MAL_S4x4_MATRIX_ACCESS_I_J(abstractMatrix, iRow, iCol) = inMatrix(iRow, iCol);
    }
  }
  return abstractMatrix;
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
    if (parentJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent, parentComponent)) {
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
    CkppJointComponentShPtr childComponent = attKppJoint->childJointComponent(inRank);
    return device->kppToHppJoint(childComponent);
  }
  
  return NULL;
}

// ==========================================================================

bool ChppJoint::addChildJoint(ChppJoint& inJoint)
{
  /*
    Check that CkppJointComponent belongs to a robot. 
    Otherwise, the joint cannot be registered in the device map (ChppDevice::attKppToHppJointMap).
  */
  if (ChppDeviceShPtr device = hppDevice()) {
    /*
      Register joint in device.
    */
    device->registerJoint(inJoint);

    if (attKppJoint->addChildJointComponent(inJoint.attKppJoint) != KD_OK) {
      return false;
    }
    if (!attJrlJoint->addChildJoint(*(inJoint.attJrlJoint))) {
      attKppJoint->removeChildJointComponent(inJoint.attKppJoint);
      return false;
    }
    return true;
  }
  return false;
}

// ==========================================================================

ChppDeviceShPtr ChppJoint::hppDevice()
{
  CkppDeviceComponentShPtr kppDevice = attKppJoint->deviceComponent();
  ChppDeviceShPtr device;

  device = KIT_DYNAMIC_PTR_CAST(ChppDevice, kppDevice);
  return device;
}


// ==========================================================================

template <class CkppJnt, class CjrlJnt> ChppJoint* ChppJoint::createJoint(std::string inName, 
									  const CkitMat4& inInitialPosition)
{
  /*
    Create ChppJoint
  */
  ChppJoint* hppJoint = new ChppJoint;
  /*
    Create kppJointComponent
  */
  CkppJointComponentShPtr kppJoint = CkppJnt::create(inName);
  if (kppJoint) {
    kppJoint->kwsJoint()->setCurrentPosition(inInitialPosition);
  } else {
    delete hppJoint;
    return NULL;
  }

  /*
    Convert homogeneous matrix to abstract matrix type matrix4d
  */
  matrix4d initialPos = abstractMatrixFromCkitMat4(inInitialPosition);
  CjrlJoint* jrlJoint = new CjrlJnt(initialPos);
  if (!jrlJoint) {
    delete jrlJoint;
    delete hppJoint;
    return NULL;
  }

  hppJoint->attKppJoint = kppJoint;
  hppJoint->attJrlJoint = jrlJoint;

  return hppJoint;
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
  inBody->joint(this);
}

// ==========================================================================

ChppBodyShPtr ChppJoint::attachedBody()  
{
  return KIT_DYNAMIC_PTR_CAST(ChppBody, kppJoint()->kwsJoint()->attachedBody());
}

// ==========================================================================

ChppJoint* ChppJoint::createFreeFlyer(std::string inName, const CkitMat4& inInitialPosition)
{
  return createJoint<CkppFreeFlyerJointComponent, CimplJointFreeFlyer>(inName, inInitialPosition);
}

// ==========================================================================

ChppJoint* ChppJoint::createRotation(std::string inName, const CkitMat4& inInitialPosition)
{
  return createJoint<CkppRotationJointComponent, CimplJointRotation>(inName, inInitialPosition);
}

// ==========================================================================

ChppJoint* ChppJoint::createTranslation(std::string inName, const CkitMat4& inInitialPosition)
{
  return createJoint<CkppTranslationJointComponent, CimplJointTranslation>(inName, inInitialPosition);
}

