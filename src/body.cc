///
/// Copyright (c) 2011 CNRS
/// Authors: Florent Lamiraux
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

#include "hpp/model/body.hh"
#include "hpp/model/joint.hh"
#include "hpp/model/exception.hh"

#define KITELAB_205004

namespace hpp {
  namespace model {

    Body::Body(std::string name) : joint_(NULL), name_(name) {
#ifdef KITELAB_205004
      std::vector<CkcdObjectShPtr> vect;
      vect.push_back(CkcdAssembly::create());
      innerObjects(vect);
#endif
    }

    BodyShPtr Body::create(const std::string& name)
    {
      Body* hppBody = new Body(name);
      BodyShPtr hppBodyShPtr(hppBody);
      BodyWkPtr hppBodyWkPtr = hppBodyShPtr;

      if (hppBody->init(hppBodyWkPtr) != KD_OK) {
	hppDout(error," error in create() ");
	hppBodyShPtr.reset();
      }
      return hppBodyShPtr;
    }

    //=========================================================================

    ktStatus Body::init(const BodyWkPtr weakPtr)
    {
      weakPtr_ = weakPtr;
      return CkwsKCDBody::init(weakPtr);
    }

    //=========================================================================

    bool
    Body::addInnerObject(const CkppSolidComponentRefShPtr& solidCompRef,
			 const CkitMat4& position,
			 bool distanceComputation)
    {
      CkppSolidComponentShPtr solidComponent =
	solidCompRef->referencedSolidComponent();

#ifdef HPP_DEBUG
      std::string innerName = solidComponent->name();
      std::string outerName;
#endif
      // Attach solid component to the joint associated to the body
      CkwsJointShPtr bodyKwsJoint = CkwsBody::joint();
      CkppJointComponentShPtr bodyKppJoint =
	KIT_DYNAMIC_PTR_CAST(CkppJointComponent, bodyKwsJoint);

      // Test that body is attached to a joint
      if (bodyKppJoint) {
	solidComponent->setAbsolutePosition(position);
	bodyKppJoint->addSolidComponentRef(solidCompRef);
      }
      else {
	hppDout(error, "The body is not attached to any joint.");
	throw Exception("The body is not attached to any joint.");
      }

      // If requested, add the object in the list of objects the
      //distance to which needs to be computed and build the
      //corresponding analyses.
      if (distanceComputation) {
	CkcdObjectShPtr innerObject =
	  KIT_DYNAMIC_PTR_CAST(CkcdObject, solidComponent);
	if (innerObject) {
	  hppDout(info,"adding " << solidComponent->name()
		  << " to list of objects for distance computation.");
	  innerObjForDist_.push_back(innerObject);
	  // Build Exact distance computation analyses for this object
	  const std::vector<CkcdObjectShPtr>& outerList = outerObjForDist_;
	  for (std::vector<CkcdObjectShPtr>::const_iterator it =
		 outerList.begin(); it != outerList.end(); it++) {
	    const CkcdObjectShPtr& outerObject = *it;

#ifdef HPP_DEBUG
	    CkppSolidComponentShPtr solidComp =
	      KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, outerObject);
	    if (solidComp) {
	      outerName = solidComp->name();
	    } else {
	      outerName = std::string("");
	    }
#endif
	    // Instantiate the analysis object
	    CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
	    analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);
	    // Ignore tolerance for distance computations
	    analysis->doesIgnoreTolerance(true);
	    // retrieve the test trees associated with the objects
	    CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
	    CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();

	    // associate the lists with the analysis object
	    analysis->leftTestTree(leftTree);
	    analysis->rightTestTree(rightTree);

	    hppDout(info,"creating analysis between "
		    << innerName << " and "
		    << outerName);
	    distCompPairs_.push_back(analysis);
	  }
	}
	else {
	  hppDout(error,"cannot cast solid component into CkcdObject.");
	  throw Exception("cannot cast solid component into CkcdObject.");
	}
      }
      return true;
    }

    //=========================================================================

    void Body::addOuterObject(const CkcdObjectShPtr& outerObject,
			      bool distanceComputation)

    {
#ifdef HPP_DEBUG
      std::string innerName;
      std::string outerName;
      CkppSolidComponentShPtr solidComp =
	KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, outerObject);
      if (solidComp) {
	outerName = solidComp->name();
      } else {
	outerName = std::string("");
      }
#endif
      // Append object at the end of KineoWorks set of outer objects
      // for collision checking
      std::vector<CkcdObjectShPtr> outerList = outerObjects();
      outerList.push_back(outerObject);
      outerObjects(outerList);

      // If distance computation is requested, build necessary CkcdAnalysis
      // objects
      if (distanceComputation) {
	// Store object in case inner objects are added a posteriori
	outerObjForDist_.push_back(outerObject);

	// Build distance computation objects (CkcdAnalysis)
	const std::vector<CkcdObjectShPtr> innerList = innerObjForDist_;
	for (std::vector<CkcdObjectShPtr>::const_iterator it =
	       innerList.begin(); it != innerList.end(); it++) {
	  const CkcdObjectShPtr& innerObject=*it;

	  // Instantiate the analysis object
	  CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
	  analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);
	  // Ignore tolerance for distance computations
	  analysis->doesIgnoreTolerance(true);

	  // retrieve the test trees associated with the objects
	  CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
	  CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();

	  // associate the lists with the analysis object
	  analysis->leftTestTree(leftTree);
	  analysis->rightTestTree(rightTree);

#ifdef HPP_DEBUG
	  solidComp = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, innerObject);
	  if (solidComp) {
	    innerName = solidComp->name();
	  } else {
	    innerName = std::string("");
	  }
#endif
	  hppDout(info,"creating analysis between "
		  << innerName << " and "
		  << outerName);

	  distCompPairs_.push_back(analysis);
	}
      }
    }

    //=========================================================================

    void Body::resetOuterObjects()
    {
      outerObjForDist_.clear();
      distCompPairs_.clear();
    }


    //=========================================================================

    ktStatus
    Body::distAndPairsOfPoints(unsigned int inPairId,
			       double& outDistance,
			       CkcdPoint& outPointBody,
			       CkcdPoint& outPointEnv)
    {
      KWS_PRECONDITION(pairId < nbDistPairs());

      CkcdAnalysisShPtr analysis = attDistCompPairs[inPairId];
      analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

      ktStatus status = analysis->compute();
      if (KD_SUCCEEDED(status)) {
	hppDout(info,"compute succeeded.");
	unsigned int nbDistances = analysis->countExactDistanceReports();
    
	if(nbDistances == 0) {
	  //no distance information available, return 0 for instance;
	  hppDout(info,"no distance report.");
	  outDistance = 0;

	  return KD_OK;
	}
	else{
      
	  CkcdExactDistanceReportShPtr distanceReport;
	  CkitPoint3 leftPoint, rightPoint;
      
	  //distances are ordered from lowest value, to highest value.
	  distanceReport = analysis->exactDistanceReport(0);
      
	  //exact distance between two lists is always stored at the first
	  //rank of Distance reports.
	  outDistance = distanceReport->distance();

	  assert(analysis->countExactDistanceReports() > 0);
	  // Get points in absolute frame(world).
	  distanceReport->getPointsAbsolute (outPointBody, outPointEnv);

	  return KD_OK;
	}

      } else {
	hppDout(error,":distAndPairsOfPoints: compute failed.");
	return KD_ERROR;
      }
      return KD_OK;
    }
  } // namespace model
} // namespace hpp
