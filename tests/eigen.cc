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

#define BOOST_TEST_MODULE EIGEN_FEATURE
#include <boost/test/unit_test.hpp>

#include <hpp/model/eigen.hh>

using hpp::model::pseudoInverse;
using hpp::model::projectorOnKernel;
using hpp::model::projectorOnKernelOfInv;
using hpp::model::matrix_t;
using hpp::model::value_type;

BOOST_AUTO_TEST_CASE(testPseudoInverse)
{
  const std::size_t rows = 4, cols = 6;
  const value_type tol = 1e-6;
  typedef Eigen::JacobiSVD <matrix_t> SVD;
  SVD svd (rows, cols, Eigen::ComputeThinU | Eigen::ComputeThinV);
  matrix_t Mpinv (cols, rows);
  for (int i = 0; i < 1000; ++i) {
    matrix_t M = matrix_t::Random (rows, cols);
    matrix_t PK (cols, cols);
    matrix_t PKinv (rows, rows);

    svd.compute (M);
    pseudoInverse <SVD> (svd, Mpinv, tol); 
    projectorOnKernel <SVD> (svd, PK, tol); 
    projectorOnKernelOfInv <SVD> (svd, PKinv, tol); 
    matrix_t Ir = M * Mpinv;
    matrix_t Ic = Mpinv * M;
    matrix_t _M = M * Ic;
    matrix_t _Mpinv = Mpinv * Ir;
    BOOST_CHECK_MESSAGE (_M.isApprox (M), "M = M * M+ * M failed");
    BOOST_CHECK_MESSAGE (_Mpinv.isApprox (Mpinv), "M+ = M+ * M * M+ failed");
    BOOST_CHECK_MESSAGE (Ir.adjoint ().isApprox (Ir), "(M * M+)* = M * M+ failed");
    BOOST_CHECK_MESSAGE (Ic.adjoint ().isApprox (Ic), "(M+ * M)* = M+ * M failed");

    BOOST_CHECK_MESSAGE (PK.isApprox (Ic), "PK = M+ * M failed");
    BOOST_CHECK_MESSAGE (PKinv.isApprox (Ir), "PKinv = M * M+ failed");
  }
}
