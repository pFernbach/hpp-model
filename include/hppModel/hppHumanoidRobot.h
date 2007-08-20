/*
 *  Copyright (c) 2007 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPPHUMANOIDROBOT_H
#define HPPHUMANOIDROBOT_H

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

KIT_PREDEF_CLASS(ChppHumanoidRobot);
/**
 \brief Humanoid robot with geometric and dynamic model 
 
 Derives from hppDevice<CimplDynamicRobot> and from an implementation CimplHumanoidDynamicRobot  of CjrlHumanoidDynamicRobot.

 The creation of the device is done by ChppDevice::create(const std::string inName). This function returns a shared pointer to the newly created object.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/

class ChppHumanoidRobot : public hppDevice, public CimplHumanoidDynamicRobot {
public:
  ~ChppHumanoidRobot();

protected:
  ChppHumanoidRobot();
};

#endif
