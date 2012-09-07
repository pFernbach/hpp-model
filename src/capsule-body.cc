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

#include <iostream>

#include <KineoWorks2/kwsJoint.h>
#include <KineoModel/kppSolidComponentRef.h>
#include <KineoModel/kppJointComponent.h>
#include <KineoKCDModel/kppKCDPolyhedron.h>
#include <KineoKCDModel/kppKCDAssembly.h>
#include <kcd2/kcdAnalysis.h>
#include <kcd2/kcdExactDistanceReport.h>
#include <kcd2/kcdPoint.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/util/debug.hh>

#include <hpp/geometry/collision/util.hh>

#include "hpp/model/capsule-body.hh"
#include "hpp/model/joint.hh"
#include "hpp/model/exception.hh"

#define KITELAB_205004

namespace hpp {
  namespace model {

    CapsuleBody::CapsuleBody (std::string name) : Body (name) {
    }

    CapsuleBodyShPtr CapsuleBody::create (const std::string& name)
    {
      CapsuleBody* capsuleBody = new CapsuleBody (name);
      CapsuleBodyShPtr capsuleBodyShPtr (capsuleBody);
      CapsuleBodyWkPtr capsuleBodyWkPtr = capsuleBodyShPtr;

      if (capsuleBody->init (capsuleBodyWkPtr) != KD_OK) {
	hppDout (error," error in create() ");
	capsuleBodyShPtr.reset();
      }
      return capsuleBodyShPtr;
    }

    //=========================================================================

    ktStatus CapsuleBody::init (const CapsuleBodyWkPtr weakPtr)
    {
      weakPtr_ = weakPtr;
      return Body::init (weakPtr);
    }

    //=========================================================================

    bool
    CapsuleBody::addInnerCapsule (const capsule_t& innerCapsule,
				  bool distanceComputation)
    {
      // Add capsule for collision checking and for distance
      // computation.
      Body::addInnerObject (innerCapsule, distanceComputation);

      // If requested, add the segment (equivalent to the capsule) in
      // the list of segments the distance to which needs to be
      // computed.
      if (distanceComputation) {
	CkppSolidComponentShPtr solidComponent
	  = KIT_DYNAMIC_PTR_CAST (CkppSolidComponent, innerCapsule);

	// if (solidComponent) {
	//   hppDout(info,"adding " << solidComponent->name ()
	// 	  << " to list of capsules for distance computation.");
	  innerCapsulesForDist_.push_back (innerCapsule);
	  // Build Exact distance computation pairs for capsule
	  const std::vector<capsule_t>& outerList = outerCapsulesForDist_;
	  for (std::vector<capsule_t>::const_iterator it =
		 outerList.begin (); it != outerList.end (); it++) {
	    const capsule_t& outerCapsule = *it;

	    // Build new collision pair between inner and outer
	    // capsules.
	    capsuleDistCompPair_t distCompPair (innerCapsule, outerCapsule);

	    hppDout(info,"creating collision pair between "
		    << innerName << " and "
		    << outerName);
	    capsuleDistCompPairs_.push_back(distCompPair);
	  }
	// }
	// else {
	//   hppDout(error,"cannot cast solid component into CkcdObject.");
	//   throw Exception("cannot cast solid component into CkcdObject.");
	// }
      }
      return true;
    }

    //=========================================================================

    void CapsuleBody::addOuterCapsule(const capsule_t& outerCapsule,
				      bool distanceComputation)

    {
      // Add capsule for collision checking but not for distance
      // computation.

      // FIXME: For now we keep adding capsule (which is in fact a
      // segment), but in reality this is unnecessary as long the user
      // adds the real capsule as outer object later.
      Body::addOuterObject (outerCapsule, false);

      // If distance computation is requested, build necessary
      // distance computation pairs.
      if (distanceComputation) {
	// Store object in case inner objects are added a posteriori
	outerCapsulesForDist_.push_back (outerCapsule);

	// Build distance computation pairs
	const std::vector<capsule_t> innerList = innerCapsulesForDist_;
	for (std::vector<capsule_t>::const_iterator it =
	       innerList.begin(); it != innerList.end(); it++) {
	  const capsule_t& innerCapsule=*it;

#ifdef HPP_DEBUG
	  solidComp = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, innerObject);
	  if (solidComp) {
	    innerName = solidComp->name();
	  } else {
	    innerName = std::string("");
	  }
#endif
	  hppDout(info,"creating distance computation pair between "
		  << innerName << " and "
		  << outerName);

	  // Build new collision pair between inner and outer
	  // capsules.
	  capsuleDistCompPair_t distCompPair (innerCapsule, outerCapsule);
	  capsuleDistCompPairs_.push_back (distCompPair);
	}
      }
    }

    //=========================================================================

    void CapsuleBody::resetOuterObjects()
    {
      Body::resetOuterObjects ();
      resetOuterCapsules ();
    }

    //=========================================================================

    void CapsuleBody::resetOuterCapsules()
    {
      outerCapsulesForDist_.clear();
      capsuleDistCompPairs_.clear();
    }


    //=========================================================================

    ktStatus
    CapsuleBody::distAndPairsOfPoints (unsigned int inPairId,
				       double& outDistance,
				       CkcdPoint& outPointBody,
				       CkcdPoint& outPointEnv)
    {
      KWS_PRECONDITION(pairId < nbDistPairs());

      if (inPairId < nbKCDDistPairs ())
	{
	  Body::distAndPairsOfPoints
	    (inPairId, outDistance, outPointBody, outPointEnv);

	  // We assume here that there is only one inner object and
	  // that it is a capsule.
	  leftRadius_ = innerCapsulesForDist_[0]->getSegmentRadius (0);

	  // Compute distance and nearest points for capsule.
	  outDistance = outDistance - leftRadius_;
	  axis_ = outPointEnv - outPointBody;
	  axis_.normalize ();
	  outPointBody = outPointBody + axis_ * leftRadius_;

	  return KD_OK;
	}
      else
	{
	  // Compute distance between two capsules with nearest points.
	  distPair_ = capsuleDistCompPairs_[inPairId - nbKCDDistPairs ()];

	  distPair_.first->getSegment (0, leftEndPoint1_, leftEndPoint2_,
				       leftRadius_);
	  distPair_.second->getSegment (0, rightEndPoint1_, rightEndPoint2_,
					rightRadius_);

	  // Apply current transformation.
	  distPair_.first->getAbsolutePosition (leftAbsPosition_);
	  distPair_.second->getAbsolutePosition (rightAbsPosition_);
	  leftEndPoint1_ = leftAbsPosition_ * leftEndPoint1_;
	  leftEndPoint2_ = leftAbsPosition_ * leftEndPoint2_;
	  rightEndPoint1_ = rightAbsPosition_ * rightEndPoint1_;
	  rightEndPoint2_ = rightAbsPosition_ * rightEndPoint2_;
	  
	  using namespace hpp::geometry::collision;
	  computeSquareDistanceSegmentSegment (leftEndPoint1_,
					       leftEndPoint2_,
					       rightEndPoint1_,
					       rightEndPoint2_,
					       squareDistance_,
					       outPointBody,
					       outPointEnv);

	  // Compute distance and nearest points for capsules.
	  outDistance = sqrt (squareDistance_) - (leftRadius_ + rightRadius_);
	  axis_ = outPointEnv - outPointBody;
	  axis_.normalize ();
	  outPointBody = outPointBody + axis_ * leftRadius_;
	  outPointEnv = outPointEnv - axis_ * rightRadius_;

	  return KD_OK;
	}
    }

    //=========================================================================

    ktStatus
    CapsuleBody::kcdDistAndPairsOfPoints (double& outDistance,
					  CkcdPoint& outPointBody,
					  CkcdPoint& outPointEnv)
    {
      double minDistance = std::numeric_limits<double>::max ();
      CkcdPoint minPointBody, minPointEnv;
      for (unsigned int i = 0; i < nbKCDDistPairs (); ++i)
	{
	  if (KD_OK == distAndPairsOfPoints (i,
					     outDistance,
					     outPointBody,
					     outPointEnv))
	    {
	      if (outDistance < minDistance)
		{
		  minDistance = outDistance;
		  minPointBody = outPointBody;
		  minPointEnv = outPointEnv;
		} 
	    }
	  else return KD_ERROR;
	}

      outDistance = minDistance;
      outPointBody = minPointBody;
      outPointEnv = minPointEnv;

      return KD_OK;
    }

    //=========================================================================

    ktStatus
    CapsuleBody::capsuleDistAndPairsOfPoints (double& outDistance,
					      CkcdPoint& outPointBody,
					      CkcdPoint& outPointEnv)
    {
      double minDistance = std::numeric_limits<double>::max ();
      CkcdPoint minPointBody, minPointEnv;
      for (unsigned int i = nbKCDDistPairs (); i < nbDistPairs (); ++i)
	{
	  if (KD_OK == distAndPairsOfPoints (i,
					     outDistance,
					     outPointBody,
					     outPointEnv))
	    {
	      if (outDistance < minDistance)
		{
		  minDistance = outDistance;
		  minPointBody = outPointBody;
		  minPointEnv = outPointEnv;
		} 
	    }
	  else return KD_ERROR;
	}

      outDistance = minDistance;
      outPointBody = minPointBody;
      outPointEnv = minPointEnv;

      return KD_OK;
    }

    //=========================================================================

    ktStatus
    CapsuleBody::distAndPairsOfPoints (double& outDistance,
				       CkcdPoint& outPointBody,
				       CkcdPoint& outPointEnv)
    {
      double minDistance = std::numeric_limits<double>::max ();
      CkcdPoint minPointBody, minPointEnv;
      for (unsigned int i = 0; i < nbDistPairs (); ++i)
	{
	  if (KD_OK == distAndPairsOfPoints (i,
					     outDistance,
					     outPointBody,
					     outPointEnv))
	    {
	      if (outDistance < minDistance)
		{
		  minDistance = outDistance;
		  minPointBody = outPointBody;
		  minPointEnv = outPointEnv;
		} 
	    }
	  else return KD_ERROR;
	}

      outDistance = minDistance;
      outPointBody = minPointBody;
      outPointEnv = minPointEnv;

      return KD_OK;
    }
  } // namespace model
} // namespace hpp
