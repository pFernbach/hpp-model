//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
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

#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fcl/math/transform.h>
#include <jrl/mathtools/angle.hh>
#include <hpp/util/debug.hh>
#include <hpp/model/joint-configuration.hh>

namespace hpp {
  namespace model {
    typedef Eigen::AngleAxis <double> AngleAxis_t;
    typedef fcl::Quaternion3f Quaternion_t;

    JointConfiguration::JointConfiguration (size_type numberDof)
    {
      bounded_.resize (numberDof);
      lowerBounds_.resize (numberDof);
      upperBounds_.resize (numberDof);
      for (size_type i=0; i < (size_type)bounded_.size (); ++i) {
	bounded_ [i] = false;
      }
    }

    JointConfiguration::~JointConfiguration ()
    {
    }

    void JointConfiguration::isBounded (size_type rank, bool bounded)
    {
      bounded_ [rank] = bounded;
    }

    bool JointConfiguration::isBounded (size_type rank) const
    {
      return bounded_ [rank];
    }

    double JointConfiguration::lowerBound (size_type rank) const
    {
      return lowerBounds_ [rank];
    }

    double JointConfiguration::upperBound (size_type rank) const
    {
      return upperBounds_ [rank];
    }

    void JointConfiguration::lowerBound (size_type rank, double lowerBound)
    {
      lowerBounds_ [rank] = lowerBound;
    }

    void JointConfiguration::upperBound (size_type rank, double upperBound)
    {
      upperBounds_ [rank] = upperBound;
    }

    AnchorJointConfig::AnchorJointConfig () : JointConfiguration (0)
    {
    }

    AnchorJointConfig::~AnchorJointConfig ()
    {
    }

    SO3JointConfig::SO3JointConfig () : JointConfiguration (6)
    {
    }

    SO3JointConfig::~SO3JointConfig ()
    {
    }

    RotationJointConfig::RotationJointConfig () : JointConfiguration (1)
    {
    }

    RotationJointConfig::~RotationJointConfig ()
    {
    }

    TranslationJointConfig::TranslationJointConfig () :
      JointConfiguration (1)
    {
    }

    TranslationJointConfig::~TranslationJointConfig ()
    {
    }

    void AnchorJointConfig::interpolate (ConfigurationIn_t,
					 ConfigurationIn_t,
					 const double&,
					 const size_type&,
					 ConfigurationOut_t)
    {
    }

    double AnchorJointConfig::distance (ConfigurationIn_t,
					ConfigurationIn_t,
					const size_type&) const
    {
      return 0;
    }

    void AnchorJointConfig::integrate (ConfigurationIn_t,
				       vectorIn_t,
				       const size_type&,
				       const size_type&,
				       ConfigurationOut_t) const
    {
    }

    void AnchorJointConfig::difference (ConfigurationIn_t,
					ConfigurationIn_t,
					const size_type&,
					const size_type&,
					vectorOut_t) const
    {
    }

    void AnchorJointConfig::uniformlySample (const size_type&,
					     ConfigurationOut_t) const
    {
    }

    /// Compute quaternion and angle from a SO(3) joint configuration
    ///
    /// \param q1, q2, robot configurations
    /// \param index index of joint configuration in robot configuration vector
    /// \param unit quaternion corresponding to both joint configuration
    /// \return angle between both joint configuration
    static double angleBetweenQuaternions (ConfigurationIn_t q1,
					   ConfigurationIn_t q2,
					   const size_type& index)
    {
      double innerprod = q1.segment (index, 4).dot (q2.segment (index, 4));
      assert (fabs (innerprod) < 1.0001);
      if (innerprod < -1) innerprod = -1;
      if (innerprod >  1) innerprod =  1;
      double theta = acos (innerprod);
      return theta;
    }

    void SO3JointConfig::interpolate (ConfigurationIn_t q1,
				      ConfigurationIn_t q2,
				      const double& u,
				      const size_type& index,
				      ConfigurationOut_t result)
    {
      // Linearly interpolate translation part
      result [index] = (1-u) * q1 [index] + u * q2 [index];
      result [index+1] = (1-u) * q1 [index+1] + u * q2 [index+1];
      result [index+2] = (1-u) * q1 [index+2] + u * q2 [index+2];
      // for rotation part, transform roll pitch yaw into quaternion
      double theta = angleBetweenQuaternions (q1, q2, index);

      if (fabs (theta) > 1e-6) {
	result.segment (index, 4) =
	  (sin ((1-u)*theta)/sin (theta)) * q1.segment (index, 4) +
	  (sin (u*theta)/sin (theta)) * q2.segment (index, 4);
      } else {
	result.segment (index, 4) =
	  (1-u) * q1.segment (index, 4) + u * q2.segment (index, 4);
      }
    }

    double SO3JointConfig::distance (ConfigurationIn_t q1,
				     ConfigurationIn_t q2,
				     const size_type& index) const
    {
      double theta = angleBetweenQuaternions (q1, q2, index);
      assert (theta >= 0);
      return theta;
    }

    void SO3JointConfig::integrate (ConfigurationIn_t q,
				    vectorIn_t v,
				    const size_type& indexConfig,
				    const size_type& indexVelocity,
				    ConfigurationOut_t result) const
    {
      vector3_t omega (v [indexVelocity + 0], v [indexVelocity + 1],
		       v [indexVelocity + 2]);

      double angle = .5*omega.norm();
      if (angle == 0) {
	result.segment (indexConfig, 4) = q.segment (indexConfig, 4);
	return;
      }
      // try to keep norm of quaternion close to 1.
      value_type norm2p = q.segment (indexConfig, 4).squaredNorm ();
      Quaternion_t p ((1.5-.5*norm2p) * q [indexConfig + 0],
		      (1.5-.5*norm2p) * q [indexConfig + 1],
		      (1.5-.5*norm2p) * q [indexConfig + 2],
		      (1.5-.5*norm2p) * q [indexConfig + 3]);
      vector3_t k = (sin (angle)/omega.norm())*omega;
      Quaternion_t pOmega (cos (angle), k [0], k [1], k [2]);
      Quaternion_t res = pOmega*p;
      // Eigen does not allow to get the 4 coefficients at once.
      result [indexConfig + 0] = res.getW ();
      result [indexConfig + 1] = res.getX ();
      result [indexConfig + 2] = res.getY ();
      result [indexConfig + 3] = res.getZ ();
    }

    void SO3JointConfig::difference (ConfigurationIn_t q1,
				     ConfigurationIn_t q2,
				     const size_type& indexConfig,
				     const size_type& indexVelocity,
				     vectorOut_t result) const
    {
      // Compute rotation vector between q2 and q1.
      Quaternion_t p1 (q1 [indexConfig + 0], q1 [indexConfig + 1],
		       q1 [indexConfig + 2], q1 [indexConfig + 3]);
      Quaternion_t p2 (q2 [indexConfig + 0], q2 [indexConfig + 1],
		       q2 [indexConfig + 2], q2 [indexConfig + 3]);
      Quaternion_t p (p1); p.conj (); p*=p2;
      double angle; fcl::Vec3f axis;
      p.toAxisAngle (axis, angle);
      result [indexVelocity + 0] = angle*axis [0];
      result [indexVelocity + 1] = angle*axis [1];
      result [indexVelocity + 2] = angle*axis [2];
    }

    void SO3JointConfig::uniformlySample (const size_type& index,
					  ConfigurationOut_t result) const
    {
      double u1 = (double)rand() / RAND_MAX;
      double u2 = (double)rand() / RAND_MAX;
      double u3 = (double)rand() / RAND_MAX;
      result [index] = sqrt (1-u1)*sin(2*M_PI*u2);
      result [index+1] = sqrt (1-u1)*cos(2*M_PI*u2);
      result [index+2] = sqrt (u1) * sin(2*M_PI*u3);
      result [index+3] = sqrt (u1) * cos(2*M_PI*u3);
    }

    void TranslationJointConfig::interpolate (ConfigurationIn_t q1,
					      ConfigurationIn_t q2,
					      const double& u,
					      const size_type& index,
					      ConfigurationOut_t result)
    {
      result [index] = (1-u) * q1 [index] + u * q2 [index];
      hppDout (info, "index = " << index);
      hppDout (info, "result [index] = " << result [index]);
    }

    double TranslationJointConfig::distance (ConfigurationIn_t q1,
					     ConfigurationIn_t q2,
					     const size_type& index) const
    {
      return fabs (q2 [index] - q1 [index]);
    }

    void TranslationJointConfig::integrate (ConfigurationIn_t q,
					    vectorIn_t v,
					    const size_type& indexConfig,
					    const size_type& indexVelocity,
					    ConfigurationOut_t result) const
    {
      assert (indexConfig < result.size ());
      result [indexConfig] = q [indexConfig] + v [indexVelocity];
      if (isBounded (0)) {
	if (result [indexConfig] < lowerBound (0)) {
	  result [indexConfig] = lowerBound (0);
	} else if (result [indexConfig] > upperBound (0)) {
	  result [indexConfig] = upperBound (0);
	}
      }
    }

    void TranslationJointConfig::difference (ConfigurationIn_t q1,
					     ConfigurationIn_t q2,
					     const size_type& indexConfig,
					     const size_type& indexVelocity,
					     vectorOut_t result) const
    {
      result [indexVelocity] = q1 [indexConfig] - q2 [indexConfig];
    }

    void TranslationJointConfig::uniformlySample (const size_type& index,
						  ConfigurationOut_t result) const
    {
      if (!isBounded (0)) {
	std::ostringstream iss
	  ("Cannot uniformly sample non bounded translation degrees of "
	   "freedom at rank ");
	iss  << index;
	throw std::runtime_error (iss.str ());
      }
      else {
	result [index] = lowerBound (0) +
	  (upperBound (0) - lowerBound (0)) * rand ()/RAND_MAX;
      }
    }

    void RotationJointConfig::interpolate (ConfigurationIn_t q1,
					   ConfigurationIn_t q2,
					   const double& u,
					   const size_type& index,
					   ConfigurationOut_t result)
    {
      if (isBounded (0)) {
	// linearly interpolate
	result [index] = (1-u) * q1 [index] + u * q2 [index];
      } else {
	// interpolate on the unit circle
	jrlMathTools::Angle th1 (q1 [index]);
	jrlMathTools::Angle th2 (q2 [index]);
	result [index] = th1.interpolate (u, th2);
      }
      hppDout (info, "index = " << index);
      hppDout (info, "result [index] = " << result [index]);
    }

    double RotationJointConfig::distance (ConfigurationIn_t q1,
					  ConfigurationIn_t q2,
					  const size_type& index) const
    {
      if (isBounded (0)) {
	// linearly interpolate
	return fabs (q2 [index] - q1 [index]);
      } else {
	// distance on the unit circle
	jrlMathTools::Angle th1 (q1 [index]);
	jrlMathTools::Angle th2 (q2 [index]);
	return th1.distance (th2);
      }
    }

    void RotationJointConfig::integrate (ConfigurationIn_t q,
					 vectorIn_t v,
					 const size_type& indexConfig,
					 const size_type& indexVelocity,
					 ConfigurationOut_t result) const
    {
      using jrlMathTools::Angle;
      double omega = v [indexVelocity];
      result [indexConfig] = Angle (q [indexConfig]) + Angle (omega);
      if (isBounded (0)) {
	if (result [indexConfig] < lowerBound (0)) {
	  result [indexConfig] = lowerBound (0);
	} else if (result [indexConfig] > upperBound (0)) {
	  result [indexConfig] = upperBound (0);
	}
      }
    }

    void RotationJointConfig::difference (ConfigurationIn_t q1,
					  ConfigurationIn_t q2,
					  const size_type& indexConfig,
					  const size_type& indexVelocity,
					  vectorOut_t result) const
    {
      using jrlMathTools::Angle;
      if (isBounded (0)) {
	result [indexVelocity] = q1 [indexConfig] - q2 [indexConfig];
      } else {
	result [indexVelocity] = Angle (q1 [indexVelocity]) -
	  Angle (q2 [indexVelocity]);
      }
    }

    void RotationJointConfig::uniformlySample (const size_type& index,
					       ConfigurationOut_t result) const
    {
      if (!isBounded (0)) {
	result [index] = -M_PI + 2* M_PI * rand ()/RAND_MAX;
      }
      else {
	result [index] = lowerBound (0) +
	  (upperBound (0) - lowerBound (0)) * rand ()/RAND_MAX;
      }
    }

  } // namespace model
} // namespace hpp
