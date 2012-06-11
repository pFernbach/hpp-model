///
/// Copyright (c) 2011 CNRS
/// Authors: Florent Lamiraux
///
///
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_MODEL_FWD_HH
# define HPP_MODEL_FWD_HH

#include <KineoUtility/kitDefine.h>
namespace hpp {
  namespace model {
    KIT_PREDEF_CLASS(Device)
    KIT_PREDEF_CLASS(Exception)
    KIT_PREDEF_CLASS(FreeflyerJoint)
    KIT_PREDEF_CLASS(HumanoidRobot)
    KIT_PREDEF_CLASS(Joint)
    KIT_PREDEF_CLASS(Body)
    KIT_PREDEF_CLASS(CapsuleBody)
    KIT_PREDEF_CLASS(BodyFactory)
    KIT_PREDEF_CLASS(CapsuleBodyFactory)
  } // namespace model
} // namespace hpp
#endif //HPP_MODEL_FWD_HH
