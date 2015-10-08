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

#ifndef HPP_MODEL_EIGEN_HH
# define HPP_MODEL_EIGEN_HH

# include <hpp/model/fwd.hh>
# include <Eigen/SVD>

namespace hpp {
  namespace model {
    template < typename SVD >
    void pseudoInverse(const SVD& svd,
        Eigen::Ref <typename SVD::MatrixType> pinvmat,
        const value_type tolerance =
        Eigen::NumTraits<typename SVD::MatrixType::Scalar>::epsilon())
    {
      eigen_assert(svd.computeU() && svd.computeV() && "Eigen::JacobiSVD "
          "computation flags must be at least: ComputeThinU | ComputeThinV");

      const typename SVD::SingularValuesType& singularValues
        = svd.singularValues ();

      typename SVD::SingularValuesType singularValues_inv =
        (singularValues.array () >= tolerance).select (
            singularValues.array ().cwiseInverse (),
            SVD::SingularValuesType::Zero (singularValues.size())
            ).matrix ();

      pinvmat = svd.matrixV () * singularValues_inv.asDiagonal()
        * svd.matrixU().adjoint();
    }

    template < typename SVD >
    void projectorOnKernel (const SVD svd,
        Eigen::Ref <typename SVD::MatrixType> projector,
        const value_type tolerance =
        Eigen::NumTraits<typename SVD::MatrixType::Scalar>::epsilon())
    {
      eigen_assert(svd.computeU() && svd.computeV() && "Eigen::JacobiSVD "
          "computation flags must be at least: ComputeThinU | ComputeThinV");

      const typename SVD::SingularValuesType& singularValues
        = svd.singularValues ();

      typename SVD::SingularValuesType sv_invTimesSv =
        (singularValues.array () >= tolerance).select (
            SVD::SingularValuesType::Ones (singularValues.size()),
            SVD::SingularValuesType::Zero (singularValues.size())
            ).matrix ();

      projector = svd.matrixV () * sv_invTimesSv.asDiagonal()
        * svd.matrixV().adjoint();
    }

    template < typename SVD >
    void projectorOnKernelOfInv (const SVD svd,
        Eigen::Ref <typename SVD::MatrixType> projector,
        const value_type tolerance =
        Eigen::NumTraits<typename SVD::MatrixType::Scalar>::epsilon())
    {
      eigen_assert(svd.computeU() && svd.computeV() && "Eigen::JacobiSVD "
          "computation flags must be at least: ComputeThinU | ComputeThinV");

      const typename SVD::SingularValuesType& singularValues
        = svd.singularValues ();

      typename SVD::SingularValuesType sv_invTimesSv =
        (singularValues.array () >= tolerance).select (
            SVD::SingularValuesType::Ones (singularValues.size()),
            SVD::SingularValuesType::Zero (singularValues.size())
            ).matrix ();

      projector = svd.matrixU () * sv_invTimesSv.asDiagonal()
        * svd.matrixU().adjoint();
    }
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_EIGEN_HH
