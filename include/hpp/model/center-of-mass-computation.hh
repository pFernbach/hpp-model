// Copyright (c) 2015, LAAS-CNRS
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

#ifndef HPP_MODEL_CENTER_OF_MASS_COMPUTATION_HH
# define HPP_MODEL_CENTER_OF_MASS_COMPUTATION_HH

# include <list>

# include <hpp/model/fwd.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace model {
    class CenterOfMassComputation
    {
      public:

        static CenterOfMassComputationPtr_t create (const DevicePtr_t& device);

        void add (const JointPtr_t& joint);

        void compute (const Device::Computation_t& flag
            = Device::ALL);

        const fcl::Vec3f& com () const
        {
          return com_;
        }

        const value_type& mass () const
        {
          return mass_;
        }

        void computeMass ();

        const ComJacobian_t& jacobian () const
        {
          return jacobianCom_;
        }

        ~CenterOfMassComputation ();

      protected:
        CenterOfMassComputation (const DevicePtr_t& device);

      private:
        // Keep a tree structure in order to compute a partial COM
        struct JointTreeElement_t;
        struct isJoint;

        typedef std::list <JointTreeElement_t*> JointTreeList;
        // JointTreeElement_t s that have no parents
        JointTreeList rootJointTrees_;
        // all JointTreeElement_t s
        JointTreeList jointTrees_;

        value_type mass_;
        vector3_t massCom_;
        vector3_t com_;
        ComJacobian_t jacobianCom_;
    }; // class CenterOfMassComputation
  }  // namespace model
}  // namespace hpp
#endif // HPP_MODEL_CENTER_OF_MASS_COMPUTATION_HH
