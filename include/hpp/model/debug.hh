//
// Copyright (c) 2015 CNRS
// Author: Joseph Mirabel
//
//
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

#ifndef HPP_MODEL_DEBUG_HH
# define HPP_MODEL_DEBUG_HH

# include <hpp/model/fwd.hh>
# include <hpp/model/config.hh>

# include <vector>
# include <string>

namespace hpp {
  namespace model {
    struct Transform {
        std::string name;
        value_type p[3];
        value_type q[4];
    };
    typedef std::vector <Transform> ForwardGeometrys_t;

    /// The device will be copyied
    static ForwardGeometrys_t forwardGeometry (DevicePtr_t device, ConfigurationIn_t q);
    static ForwardGeometrys_t forwardGeometry (DevicePtr_t device, Configuration_t q);
    static ForwardGeometrys_t forwardGeometry (DevicePtr_t device, ConfigurationOut_t q);
    static ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, ConfigurationOut_t q);
    static ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, ConfigurationIn_t q);
    static ForwardGeometrys_t forwardGeometry (HumanoidRobotPtr_t device, Configuration_t q);
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_BODY_HH
