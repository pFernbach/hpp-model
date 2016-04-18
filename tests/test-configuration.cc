///
/// Copyright (c) 2011 CNRS
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

// This test
//   - builds a robot with various types of joints,
//   - randomly samples pairs of configurations,
//   - test consistency between hpp::model::difference and
//     hpp::model::integrate functions.

#include "test-tools.hh"

BOOST_AUTO_TEST_CASE(difference_and_integrate)
{
  DevicePtr_t robot = createRobot ();
  Configuration_t q0; q0.resize (robot->configSize ());
  Configuration_t q1; q1.resize (robot->configSize ());
  Configuration_t q2; q2.resize (robot->configSize ());
  vector_t q1_minus_q0; q1_minus_q0.resize (robot->numberDof ());
  const value_type eps_dist = robot->numberDof() * sqrt(Eigen::NumTraits<value_type>::epsilon());
  for (size_type i=0; i<10000; ++i) {
    shootRandomConfig (robot, q0);
    shootRandomConfig (robot, q1);

    hpp::model::difference (robot, q1, q0, q1_minus_q0);
    hpp::model::integrate (robot, q0, q1_minus_q0, q2);

    // Check that the distance is the norm of the difference
    value_type distance = hpp::model::distance (robot, q0, q1);
    BOOST_CHECK_MESSAGE (distance - q1_minus_q0.norm () < Eigen::NumTraits<value_type>::dummy_precision(),
        "\nThe distance is not the norm of the difference\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "distance=" << distance  << "\n" <<
        "(q1 - q0).norm () = " << q1_minus_q0.norm ()
        );

    // Check that distance (q0 + (q1 - q0), q1) is zero
    distance = hpp::model::distance (robot, q1, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + (q1 - q0)) is not equivalent to q1\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );
  }
}

BOOST_AUTO_TEST_CASE(interpolate)
{
  DevicePtr_t robot = createRobot ();
  Configuration_t q0; q0.resize (robot->configSize ());
  Configuration_t q1; q1.resize (robot->configSize ());
  Configuration_t q2; q2.resize (robot->configSize ());
  vector_t q1_minus_q0; q1_minus_q0.resize (robot->numberDof ());
  const value_type eps_dist = robot->numberDof() * sqrt(Eigen::NumTraits<value_type>::epsilon());
  value_type distance;
  for (size_type i=0; i<10000; ++i) {
    shootRandomConfig (robot, q0);
    shootRandomConfig (robot, q1);

    hpp::model::interpolate (robot, q0, q1, 0, q2);
    distance = hpp::model::distance (robot, q0, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + 0 * (q1 - q0)) is not equivalent to q0\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );

    const size_type rso3 = 4;
    vector_t errors = interpolation (robot, q0, q1, 4);
    BOOST_CHECK_MESSAGE (errors.isZero (),
        "The interpolation computed by HPP does not match the Eigen SLERP"
        );

    hpp::model::interpolate (robot, q0, q1, 1, q2);
    distance = hpp::model::distance (robot, q1, q2);
    BOOST_CHECK_MESSAGE (distance < eps_dist,
        "\n(q0 + 1 * (q1 - q0)) is not equivalent to q1\n" <<
        "q0=" << q0.transpose () << "\n" <<
        "q1=" << q1.transpose () << "\n" <<
        "q2=" << q2.transpose () << "\n" <<
        "distance=" << distance
        );
  }
}

BOOST_AUTO_TEST_CASE(toEigenFunction)
{
  // Compilation check only.

  using namespace hpp::model;
  vector3_t fcl_v;
  matrix3_t fcl_m;
  fcl_v.setValue (1);
  fcl_m.setValue (3);

  vector_t eigen_v(3);
  matrix_t eigen_m(3,3);

  toEigen (fcl_v, eigen_v);
  toEigen (fcl_m, eigen_m);
  toEigen (fcl_v, eigen_m.col(0));
}
