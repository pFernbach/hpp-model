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

#include "hpp/model/center-of-mass-computation.hh"

#include <algorithm>
#include <queue>

#include "hpp/model/joint.hh"
#include "hpp/model/device.hh"

namespace hpp {
  namespace model {
    struct CenterOfMassComputation::JointTreeElement_t {
      JointTreeElement_t* parent;
      JointPtr_t j;
      fcl::Vec3f massCom_;
      std::vector <JointTreeElement_t*> children;
      JointTreeElement_t (const JointPtr_t& joint) :
        parent (NULL), j (joint), children () {
          massCom_.setValue (0);
        }
    };

    struct CenterOfMassComputation::isJoint {
      JointPtr_t j;
      isJoint (const JointPtr_t& jt) : j (jt) {}

      bool operator() (const JointTreeElement_t* e)
      {
        return e->j == j;
      }
    };

    CenterOfMassComputationPtr_t CenterOfMassComputation::create (
        const DevicePtr_t& d)
    {
      return CenterOfMassComputationPtr_t (new CenterOfMassComputation (d));
    }

    void CenterOfMassComputation::computeMass ()
    {
      mass_ = 0;
      for (JointTreeList::iterator it = rootJointTrees_.begin ();
          it != rootJointTrees_.end (); ++it) {
        mass_ += (*it)->j->computeMass ();
      }
      assert (mass_ > 0);
    }

    void CenterOfMassComputation::compute (const Device::Computation_t& flag)
    {
      assert (mass_ > 0);
      if (flag & Device::COM) {
        massCom_.setValue (0);
        for (JointTreeList::iterator it = rootJointTrees_.begin ();
            it != rootJointTrees_.end (); ++it) {
          (*it)->j->computeMassTimesCenterOfMass ();
          massCom_ += (*it)->j->massCom_;
        }
        com_ = massCom_ / mass_;
      }
      if (flag & Device::JACOBIAN) {
        jacobianCom_.setZero ();
        for (JointTreeList::iterator it = jointTrees_.begin ();
            it != jointTrees_.end (); ++it)
          (*it)->j->writeComSubjacobian (jacobianCom_, mass_);
      }
    }

    CenterOfMassComputation::CenterOfMassComputation (const DevicePtr_t& d) :
      rootJointTrees_ (), jointTrees_ (), mass_ (-1),
      jacobianCom_ (3, d->numberDof ())
    {
      massCom_.setZero ();
    }

    void CenterOfMassComputation::add (const JointPtr_t& j)
    {
      JointTreeList::iterator itj =
        std::find_if (jointTrees_.begin (), jointTrees_.end (), isJoint (j));
      if (itj == jointTrees_.end ()) {
        // TODO: Here we consider that all added joint are root joints. It
        // means that you must consider all joints below this joints.
        // A tree structure should be built to allow a finer selection of
        // joints.
        JointTreeElement_t* jtree = new JointTreeElement_t (j);
        rootJointTrees_.push_back (jtree);
        jointTrees_.push_back (jtree);
        std::queue <JointTreeElement_t*> q;
        q.push (jtree);
        while (!q.empty ()) {
          JointTreeElement_t* current = q.front ();
          for (size_t r = 0; r < current->j->numberChildJoints (); ++r) {
            JointTreeElement_t* next =
              new JointTreeElement_t (current->j->childJoint (r));
            next->parent = current;
            current->children.push_back (next);
            jointTrees_.push_back (next);
            q.push (next);
          }
          q.pop ();
        }
      }
    }

    CenterOfMassComputation::~CenterOfMassComputation ()
    {
      for (JointTreeList::iterator it = jointTrees_.begin ();
          it != jointTrees_.end ();) {
        delete *it;
        it = jointTrees_.erase (it);
      }
    }
  }  //  namespace model
}  //  namespace hpp
