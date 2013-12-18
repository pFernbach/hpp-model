///
/// Copyright (c) 2013, 2014 CNRS
/// Author: Florent Lamiraux
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

#ifndef HPP_MODEL_DISTANCE_RESULT_HH
# define HPP_MODEL_DISTANCE_RESULT_HH

# include <fcl/collision_data.h>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>

namespace hpp {
  namespace model {
    struct HPP_MODEL_DLLAPI DistanceResult {
      fcl::DistanceResult fcl;
      CollisionObjectShPtr innerObject;
      CollisionObjectShPtr outerObject;
    }; // struct DistanceResult
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_DISTANCE_RESULT_HH
