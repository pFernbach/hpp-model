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

#ifndef HPP_MODEL_FCL_TO_EIGEN_HH
# define HPP_MODEL_FCL_TO_EIGEN_HH

# include <fcl/math/transform.h>
# include <hpp/model/fwd.hh>

inline hpp::model::matrix_t operator* (const hpp::model::matrix_t& m1,
				       const fcl::Matrix3f& m2)
{
  assert (m1.cols () == 3);
  std::size_t rows = m1.rows ();
  hpp::model::matrix_t result (rows, 3);
  for (std::size_t i=0; i<rows; ++i) {
    for (std::size_t k=0; k<3; ++k) {
      result (i, k) = 0;
      for (std::size_t j=0; j<3; ++j) { 
	result (i, k) += m1 (i,j) * m2 (j, k);
      }
    }
  }
  return result;
}

namespace hpp {
  namespace model {
    inline void toEigen (const hpp::model::vector3_t& v,
			 hpp::model::vector_t& res)
    {
      res [0] = v [0]; res [1] = v [1]; res [2] = v [2];
    }
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_FCL_TO_EIGEN_HH
