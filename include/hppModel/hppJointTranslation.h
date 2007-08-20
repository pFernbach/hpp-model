/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux
 */

#ifndef HPPJOINTTRANSLATION_H
#define HPPJOINTTRANSLATION_H

/**
   \brief Free-flyer joint of a ChppDevice.

   Derives from
   \li a CimplJointTranslation implementation of CjrlJoint,
   \li ChppJoint,
   \li CkwsJointTranslation.
*/

class ChppJointTranslation : public CimplJointTranslation, public ChppJoint, public CkppTranslationJointComponent {
public:
  ~ChppJointTranslation();
protected:
  ChppJointTranslation();
};
