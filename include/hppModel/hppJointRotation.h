/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINTROTATION_H
#define HPPJOINTROTATION_H

/**
   \brief Free-flyer joint of a ChppDevice.

   Derives from
   \li CimplJointRotation: a rotation implementation of CjrlJoint,
   \li ChppJoint,
   \li CkwsJointRotation.
*/

class ChppJointRotation : public CimplJointRotation, public ChppJoint, public CkwsRotationJointComponent {
public:
  ~ChppJointRotation();
protected:
  ChppJointRotation();
};
