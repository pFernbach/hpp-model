///
/// Copyright (c) 2012 CNRS
/// Authors: Antonio El Khoury
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

#ifndef HPP_MODEL_CAPSULE_BODY_HH
#define HPP_MODEL_CAPSULE_BODY_HH

/*************************************
INCLUDE
**************************************/

#include <KineoUtility/kitDefine.h>
#include <kcd2/kcdAnalysisType.h>

#include <hpp/geometry/component/segment.hh>

#include "hpp/model/fwd.hh"
#include "hpp/model/body.hh"

namespace hpp {
  namespace model {

    /// \brief Specialization of a body representing a capsule
    /// geometric object attached to a joint.
    class CapsuleBody : public Body
    {
    public:

      typedef hpp::geometry::component::SegmentShPtr capsule_t;
      typedef std::pair<capsule_t, capsule_t> capsuleDistCompPair_t;

      /// \brief Creation of a body
      /// \param name Name of the new body.
      /// \return A shared pointer to a new body.
      static CapsuleBodyShPtr create (const std::string& name);

      /// \name Define inner and outer objects
      /// @{
      /// \brief Add a geometric object to the body

      /// \param solidCompRef Reference to the solid component to add.
      /// \param position Position of the object before attaching it to the body
      /// (default value=Identity).
      /// \param distanceComputation whether this object should be put in the
      /// distance computation analysis.
      /// \return true if success, false otherwise.
      /// The object is added to the inner object list of the body.
      /// \note The body must be attached to a joint.
      bool addInnerCapsule (const CkppSolidComponentRefShPtr& solidCompRef,
			    const CkitMat4& position=CkitMat4(),
			    bool distanceComputation=false);

      /// \param innerCapsule capsule to add.
      /// \param distanceComputation whether this object should be put
      /// in the distance computation analysis.
      /// \return true if success, false otherwise.
      /// The object is added to the inner object list of the body.
      /// \note The body must be attached to a joint.
      /// \note IF the object is already added, put it only in a
      /// distance computation analysis.
      bool addInnerCapsule (const capsule_t& innerCapsule,
			    bool distanceComputation=true);

      /// \brief Add a capsule for collision testing with the body
      /// \param outerCapsule new capsule
      /// \param distanceComputation whether distance analyses should be added for
      /// this object.
      void addOuterCapsule (const capsule_t& outerObject,
			    bool distanceComputation=true);

      /// \brief Reset the list of outer capsules
      void resetOuterCapsules ();

      /// \brief Reset the list of outer objects
      void resetOuterObjects ();

      ///
      /// @}
      ///

      /// \name Distance computation
      /// @{

      /// \brief Get the number of pairs of object for which distance
      /// is computed
      unsigned int nbDistPairs () { return Body::nbDistPairs ()
	  + capsuleDistCompPairs_.size (); };

      /// \brief Get the number of kcd pairs of object for which
      /// distance is computed
      unsigned int nbKCDDistPairs () { return Body::nbDistPairs (); };

      /// \brief Get the number of pairs of capsules for which
      /// distance is computed
      unsigned int nbCapsuleDistPairs ()
      { return capsuleDistCompPairs_.size(); };

      /// \brief Compute exact distance and closest points between
      /// body and set of outer objects.

      /// \param pairId id of the pair of objects

      /// \retval outDistance Distance between body and outer objects
      /// \retval outPointCapsuleBody Closest point on body (in global reference frame)
      /// \retval outPointEnv Closest point in outer object set (in global reference frame)
      ktStatus distAndPairsOfPoints (unsigned int pairId,
				     double& outDistance,
				     CkcdPoint& outPointBody,
				     CkcdPoint& outPointEnv);

      /// \brief Compute minimum exact distance and closest points
      /// between body and set of outer KCD objects.

      /// \retval outDistance Distance between body and outer KCD objects
      /// \retval outPointCapsuleBody Closest point on body (in global reference frame)
      /// \retval outPointEnv Closest point in outer kcd object set (in global reference frame)
      ktStatus kcdDistAndPairsOfPoints (double& outDistance,
					CkcdPoint& outPointBody,
					CkcdPoint& outPointEnv);

      /// \brief Compute minimum exact distance and closest points
      /// between body and set of outer capsules.

      /// \retval outDistance Distance between body and outer capsules
      /// \retval outPointCapsuleBody Closest point on body (in global reference frame)
      /// \retval outPointEnv Closest point in outer capsule object set (in global reference frame)
      ktStatus capsuleDistAndPairsOfPoints (double& outDistance,
					    CkcdPoint& outPointBody,
					    CkcdPoint& outPointEnv);

      /// \brief Compute minimum exact distance and closest points
      /// between body and set of outer objects.

      /// \retval outDistance Distance between body and outer objects
      /// \retval outPointCapsuleBody Closest point on body (in global reference frame)
      /// \retval outPointEnv Closest point in outer object set (in global reference frame)
      ktStatus distAndPairsOfPoints (double& outDistance,
				     CkcdPoint& outPointBody,
				     CkcdPoint& outPointEnv);

      ///
      /// @}
      ///

    protected:

      /// \brief Constructor by name.
      CapsuleBody (std::string name);

      /// \brief Initialization of body
      /// \param weakPtr weak pointer to itself
      ktStatus init (const CapsuleBodyWkPtr weakPtr);

    private:

      /// \brief Inner capsules for which distance computation is performed
      std::vector<capsule_t> innerCapsulesForDist_;

      /// \brief Outer capsules for which distance computation is performed
      std::vector<capsule_t> outerCapsulesForDist_;

      /// \brief Capsule collision pairs for this body
      /// Each pair (inner capsule, outer capsule) potentially defines
      /// an exact distance analysis.
      std::vector<capsuleDistCompPair_t> capsuleDistCompPairs_;

      /// \brief Weak pointer to itself
      CapsuleBodyWkPtr weakPtr_;

      /// \brief Temporary variables used in distance computation.
      mutable capsuleDistCompPair_t distPair_;
      mutable CkcdPoint leftEndPoint1_;
      mutable CkcdPoint leftEndPoint2_;
      mutable CkcdPoint rightEndPoint1_;
      mutable CkcdPoint rightEndPoint2_;
      mutable kcdReal leftRadius_;
      mutable kcdReal rightRadius_;
      mutable CkcdMat4 leftAbsPosition_;
      mutable CkcdMat4 rightAbsPosition_;
      mutable kcdReal squareDistance_;
      mutable CkitVect3 axis_;

    }; // class CapsuleBody
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_CAPSULE_BODY_HH
