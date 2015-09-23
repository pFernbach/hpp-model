// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-model.
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
// hpp-model. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/model/debug.hh>

#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/device.hh>
#include <hpp/model/humanoid-robot.hh>

namespace hpp {
  namespace model {
    ForwardGeometrys_t forwardGeometry (DevicePtr_t device, ConfigurationIn_t q)
    {
      ForwardGeometrys_t fgm;
      DevicePtr_t d = device->clone ();

      d->controlComputation (Device::JOINT_POSITION);
      d->currentConfiguration (q);
      d->computeForwardKinematics ();

      std::string rn = device->name () + "/";
      Transform pos;
      const JointVector_t& jv = d->getJointVector ();
      JointVector_t::const_iterator itJ;
      for (itJ = jv.begin (); itJ != jv.end (); ++itJ) {
        BodyPtr_t b = (*itJ)->linkedBody ();
        if (b == NULL) continue;
        pos.name = rn + b->name();
        Transform3f t = (*itJ)->currentTransformation () *
          (*itJ)->linkInJointFrame ();
        for (int i = 0; i < 3; ++i) pos.p[i] = t.getTranslation  ()[i];
        for (int i = 0; i < 4; ++i) pos.q[i] = t.getQuatRotation ()[i];
        fgm.push_back (pos);
      }
      return fgm;
    }

    ForwardGeometrys_t forwardGeometry (DevicePtr_t device, Configuration_t q)
    {
      return forwardGeometry (device, (ConfigurationIn_t)q);
    }

    ForwardGeometrys_t forwardGeometry (DevicePtr_t device, ConfigurationOut_t q)
    {
      return forwardGeometry (device, (ConfigurationIn_t)q);
    }

    ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, ConfigurationOut_t q)
    {
      return forwardGeometry (HPP_STATIC_PTR_CAST(Device, device), (ConfigurationIn_t)q);
    }

    ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, ConfigurationIn_t q)
    {
      return forwardGeometry (HPP_STATIC_PTR_CAST(Device, device), (ConfigurationIn_t)q);
    }

    ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, Configuration_t q)
    {
      return forwardGeometry (HPP_STATIC_PTR_CAST(Device, device), (ConfigurationIn_t)q);
    }
  } // namespace model
} // namespace hpp
