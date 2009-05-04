
/*

  Copyright 2007 LAAS-CNRS
  Author: Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>

#include "KineoModel/kppSolidComponentRef.h"
#include "KineoKCDModel/kppKCDPolyhedron.h"
#include "KineoKCDModel/kppKCDAssembly.h"
#include "kcd2/kcdAnalysis.h"

#include "hppModel/hppBody.h"
#include "hppModel/hppJoint.h"

#if 0
#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitDefine.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppSolidComponentRef.h"
#endif

#if DEBUG==2
#undef NDEBUG
#define ODEBUG2(x) std::cout << "ChppBody:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppBody:" << x << std::endl
#elif DEBUG==1
#undef NDEBUG
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppBody:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

//=============================================================================

ChppBodyShPtr ChppBody::create(std::string inName)
{
  ChppBody* hppBody = new ChppBody(inName);
  ChppBodyShPtr hppBodyShPtr(hppBody);
  ChppBodyWkPtr hppBodyWkPtr = hppBodyShPtr;
  
  if (hppBody->init(hppBodyWkPtr) != KD_OK) {
    ODEBUG1(" error in create() ");
    hppBodyShPtr.reset();
  }
  return hppBodyShPtr;
}

//=============================================================================

ktStatus ChppBody::init(const ChppBodyWkPtr inBodyWkPtr)
{

  attExactAnalyzer = CkcdAnalysis::create();

  // attExactAnalyzer->setAnalysisType(CkcdAnalysis::EXHAUSTIVE_BOOLEAN_COLLISION);
  attExactAnalyzer->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

  attWeakPtr = inBodyWkPtr;
  return CkwsKCDBody::init(inBodyWkPtr);
}

//=============================================================================

bool 
ChppBody::addInnerObject(const CkppSolidComponentRefShPtr& inSolidComponentRef, 
			 const CkitMat4& inPosition, 
			 bool inDistanceComputation)
{
  ODEBUG2(":addInnerObject: inDistanceComputation=" << inDistanceComputation);

  CkppSolidComponentShPtr solidComponent = inSolidComponentRef->referencedSolidComponent();
  
  /*
    Attach solid component to the joint associated to the body
  */
  CkwsJointShPtr bodyKwsJoint = CkwsBody::joint();
  CkppJointComponentShPtr bodyKppJoint = 
    KIT_DYNAMIC_PTR_CAST(CkppJointComponent, bodyKwsJoint);
  
  // Test that body is attached to a joint
  if (bodyKppJoint) {
    solidComponent->setAbsolutePosition(inPosition);
    bodyKppJoint->addSolidComponentRef(inSolidComponentRef);
  }
  else {
    ODEBUG1("ChppBody::addSolidComponent: the body is not attached to any joint");
    return false;
  }
  
  /*
    If requested, add the object in the list of objects the distance to which 
    needs to be computed and build the corresponding analyses.
  */
  if (inDistanceComputation) {
    CkcdObjectShPtr innerObject = 
      KIT_DYNAMIC_PTR_CAST(CkcdObject, solidComponent);
    if (innerObject) {
      ODEBUG2(":addInnerObject: adding " << solidComponent->name()
	      << " to list of objects for distance computation.");
      attInnerObjForDist.push_back(innerObject);
      /*
	Build Exact distance computation analyses for this object
      */
      const std::vector<CkcdObjectShPtr>& outerList = attOuterObjForDist;
      for (std::vector<CkcdObjectShPtr>::const_iterator it = outerList.begin();
	   it < outerList.end(); it++) {
	const CkcdObjectShPtr& outerObject = *it;

	/* Instantiate the analysis object */
	CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
	analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

	/* retrieve the test trees associated with the objects */
	CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
	CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();

	// associate the lists with the analysis object
	analysis->leftTestTree(leftTree);
	analysis->rightTestTree(rightTree);
	
	attDistCompPairs.push_back(analysis);
      }
    }
    else {
      ODEBUG1("addInnerObject: cannot cast solid component into CkcdObject.");
    }
  }
  return true;
}

void ChppBody::addOuterObject(const CkwsBodyShPtr& inBody,
			      const CkcdObjectShPtr& inOuterObject, 
			      bool inDistanceComputation)
{
  if (ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, inBody)) {
    ODEBUG2(":addOuterObject: ChppBody type.");
    hppBody->addOuterObject(inOuterObject, inDistanceComputation);
  }
  else if (CkwsKCDBodyShPtr kcdBody = 
	   KIT_DYNAMIC_PTR_CAST(CkwsKCDBody, inBody)) {
    ODEBUG2(":addOuterObject: CkwsKCDBody type.");
    /*
      Append object at the end of KineoWorks set of outer objects
      for collision checking
    */
    std::vector<CkcdObjectShPtr> outerList = kcdBody->outerObjects();
    outerList.push_back(inOuterObject);
    kcdBody->outerObjects(outerList);
  }
}

//=============================================================================

void ChppBody::addOuterObject(const CkcdObjectShPtr& inOuterObject, 
			      bool inDistanceComputation)

{
  /*
    Append object at the end of KineoWorks set of outer objects
    for collision checking
  */
  std::vector<CkcdObjectShPtr> outerList = outerObjects();
  outerList.push_back(inOuterObject);
  outerObjects(outerList);
    
  /**
     If distance computation is requested, build necessary CkcdAnalysis 
     objects
  */
  if (inDistanceComputation) {
    /*
      Store object in case inner objects are added a posteriori
    */
    attOuterObjForDist.push_back(inOuterObject);

    /*
      Build distance computation objects (CkcdAnalysis)
    */
    const CkcdObjectShPtr& outerObject=inOuterObject;
    const std::vector<CkcdObjectShPtr> innerList = attInnerObjForDist;
    for (std::vector<CkcdObjectShPtr>::const_iterator it = innerList.begin();
	 it < innerList.end();
	 it++) {
      const CkcdObjectShPtr& innerObject=*it;

      /* Instantiate the analysis object */
      CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
      analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

      /* retrieve the test trees associated with the objects */
      CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
      CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();
      
      // associate the lists with the analysis object
      analysis->leftTestTree(leftTree);
      analysis->rightTestTree(rightTree);
      
      attDistCompPairs.push_back(analysis);
    }
  }
}

//=============================================================================

void ChppBody::resetOuterObjects()
{
  attOuterObjForDist.clear();
  attDistCompPairs.clear();
}


//=============================================================================

ktStatus 
ChppBody::distAndPairsOfPoints(unsigned int inPairId,
			       double& outDistance, 
			       CkitPoint3& outPointBody, 
			       CkitPoint3& outPointEnv,
			       CkcdObjectShPtr &outObjectBody, 
			       CkcdObjectShPtr &outObjectEnv, 
			       CkcdAnalysisType::Type inType)
{
  KWS_PRECONDITION(inType < nbDistPairs());

  const CkcdAnalysisShPtr& analysis = attDistCompPairs[inPairId];

  analysis->analysisType(inType);

  analysis->compute();
  unsigned int nbDistances = analysis->countExactDistanceReports();
      
  if(nbDistances == 0) {
    //no distance information available, return 0 for instance;
    outDistance = 0;

    outObjectBody.reset(); 
    outObjectEnv.reset();

    return KD_ERROR;
  } 
  else{

    CkcdExactDistanceReportShPtr distanceReport;
    CkitPoint3 leftPoint, rightPoint;

    //distances are ordered from lowest value, to highest value. 
    distanceReport = analysis->exactDistanceReport(0); 

    //exact distance between two lists is always stored at the first rank of Distance reports.
    outDistance = distanceReport->distance();

    if(distanceReport->countPairs()>0)
      {
	CkitMat4 firstPos, secondPos;
	// get points in the frame of the object
	distanceReport->getPoints(0,leftPoint,rightPoint) ;
        // get geometry
	outObjectBody = distanceReport->pair(0).first->geometry();
	outObjectEnv = distanceReport->pair(0).second->geometry();
	outObjectBody->getAbsolutePosition(firstPos);
	outObjectEnv->getAbsolutePosition(secondPos);
        //from these geometry points we can get points in absolute frame(world) or relative frame(geometry).
	//here we want points in the absolute frame.
	outPointBody= firstPos * leftPoint;
	outPointEnv = secondPos * rightPoint;
      }
 
    // get object
    outObjectBody = distanceReport->leftCollisionEntity();
    outObjectEnv = distanceReport->rightCollisionEntity();
   
    return KD_OK;
  }

  return KD_OK;
}

//=============================================================================

void ChppBody::setInnerObjects (const std::vector<CkcdObjectShPtr> &inInnerObjects)
{
  innerObjects(inInnerObjects);

  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree = CkcdTestTree::collectTestTrees(inInnerObjects, std::vector<CkcdObjectShPtr>());

  attExactAnalyzer->leftTestTree(tree);
}

//=============================================================================

void ChppBody::setInnerObjects (const std::vector< CkcdObjectShPtr > &inInnerObjects, 
				const std::vector< CkitMat4 > &inPlacementVector)
{
  innerObjects(inInnerObjects, inPlacementVector);

  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree = CkcdTestTree::collectTestTrees(inInnerObjects, std::vector<CkcdObjectShPtr>());

  attExactAnalyzer->leftTestTree(tree);
}

//=============================================================================

void ChppBody::setOuterObjects (const std::vector<CkcdObjectShPtr> &inOuterObjects)
{
  outerObjects(inOuterObjects);

  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree = CkcdTestTree::collectTestTrees(inOuterObjects, std::vector< CkcdObjectShPtr >());

  attExactAnalyzer->rightTestTree(tree);
}


//=============================================================================

bool 
ChppBody::addSolidComponent(const CkppSolidComponentRefShPtr& inSolidComponentRef,
			    const CkitMat4& inPosition,
			    bool inDistanceComputation)
{
  return 
    addInnerObject(inSolidComponentRef, inPosition, inDistanceComputation);
}

//=============================================================================

ktStatus ChppBody::getExactDistance(double& outDistance, CkitPoint3& outPointBody, CkitPoint3& outPointEnv,
				    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)

{
  attExactAnalyzer->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

  attExactAnalyzer->compute();
  unsigned int nbDistances = attExactAnalyzer->countExactDistanceReports();
      
  if(nbDistances == 0) {
    //no distance information available, return 0 for instance;
    outDistance = 0;

    outObjectBody.reset(); 
    outObjectEnv.reset();

    return KD_ERROR;
  } 
  else{

    CkcdExactDistanceReportShPtr distanceReport;
    CkitPoint3 leftPoint, rightPoint;

    //distances are ordered from lowest value, to highest value. 
    distanceReport = attExactAnalyzer->exactDistanceReport(0); 

    //exact distance between two lists is always stored at the first rank of Distance reports.
    outDistance = distanceReport->distance();

    if(distanceReport->countPairs()>0)
      {
	CkitMat4 firstPos, secondPos;
	// get points in the frame of the object
	distanceReport->getPoints(0,leftPoint,rightPoint) ;
        // get geometry
	outObjectBody = distanceReport->pair(0).first->geometry();
	outObjectEnv = distanceReport->pair(0).second->geometry();
	outObjectBody->getAbsolutePosition(firstPos);
	outObjectEnv->getAbsolutePosition(secondPos);
        //from these geometry points we can get points in absolute frame(world) or relative frame(geometry).
	//here we want points in the absolute frame.
	outPointBody= firstPos * leftPoint;
	outPointEnv = secondPos * rightPoint;
      }
 
    // get object
    outObjectBody = distanceReport->leftCollisionEntity();
    outObjectEnv = distanceReport->rightCollisionEntity();
   
    return KD_OK;
  }

}

//=============================================================================

ktStatus ChppBody::getEstimatedDistance(double &outDistance, 
					CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)
{
  attExactAnalyzer->analysisType(CkcdAnalysisType::ESTIMATED_DISTANCE);
  attExactAnalyzer->compute();

  unsigned int nbDistances = attExactAnalyzer->countEstimatedDistanceReports();
      
  if(nbDistances == 0) {
    //no distance information available, return 0 for instance;
    outDistance = 0;

    outObjectBody.reset(); 
    outObjectEnv.reset();

    return KD_ERROR;
  } 
  else{

    CkcdEstimatedDistanceReportShPtr distanceReport;
  

    //distances are ordered from lowest value, to highest value. 
    distanceReport = attExactAnalyzer->estimatedDistanceReport(0); 

    //exact distance between two lists is always stored at the first rank of Distance reports.
    outDistance = distanceReport->distance();

    // no points for estimated distance
    
    // get object
    outObjectBody = distanceReport->leftCollisionEntity();
    outObjectEnv = distanceReport->rightCollisionEntity();
  

    return KD_OK;
  }

}

//=============================================================================

bool ChppBody::getCollisionVec(unsigned int &outNbCollision,
			       std::vector<CkcdObjectShPtr> &outObjectBodyVector, 
			       std::vector<CkcdObjectShPtr> &outObjectEnvVector)
{
  //here we consider that collision lists (CkcdCollisionList) have been given to the analysis

  attExactAnalyzer->analysisType(CkcdAnalysisType::EXHAUSTIVE_BOOLEAN_COLLISION);
  // attExactAnalyzer->analysisType(CkcdAnalysis::FAST_BOOLEAN_COLLISION);

  attExactAnalyzer->compute();
  outNbCollision = attExactAnalyzer->countCollisionReports();

  outObjectBodyVector.clear();
  outObjectEnvVector.clear();

  if(outNbCollision > 0) {
    //there is at least one collision, get more information on that collision
    CkcdCollisionReportShPtr collisionReport;
    CkcdObjectShPtr objectBody, objectEnv;

    for(unsigned int i=0; i<outNbCollision; i++){
      
      collisionReport = attExactAnalyzer->collisionReport(i);
        
      //retrieve colliding objects
      objectBody = collisionReport->pair(i).first->geometry() ;
      objectEnv = collisionReport->pair(i).second->geometry() ;
      //here we could obtain more information, like which triangles are colliding.
      //See CkcdPolyhedronTriangle for more information

      outObjectBodyVector.push_back(objectBody);
      outObjectEnvVector.push_back(objectEnv);
    }
    return true;
  }

  return false;

}

//=============================================================================

bool ChppBody::getCollision(unsigned int &outNbCollision,
			    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)
{
  //here we consider that collision lists (CkcdCollisionList) have been given to the analysis

  // attExactAnalyzer->analysisType(CkcdAnalysis::EXHAUSTIVE_BOOLEAN_COLLISION);
  attExactAnalyzer->analysisType(CkcdAnalysisType::FAST_BOOLEAN_COLLISION);
  attExactAnalyzer->compute();
  outNbCollision = attExactAnalyzer->countCollisionReports();

  if(outNbCollision > 0) {
    //there is at least one collision, get more information on that collision
    CkcdCollisionReportShPtr collisionReport;

    collisionReport = attExactAnalyzer->collisionReport(0);

    //retrieve colliding objects
    outObjectBody = collisionReport->pair(0).first->geometry() ;
    outObjectEnv = collisionReport->pair(0).second->geometry() ;
    //here we could obtain more information, like which triangles are colliding.
    //See CkcdPolyhedronTriangle for more information

    return true;
  }

  else{
    outObjectBody.reset();
    outObjectEnv.reset();

    return false;
  }
}

//============================================================================

// ==========================================================================
// return minimum distance.
// collision: dist = 0;
// otherwise: smallest distance
double ChppBody::getMinDistance()
{
  unsigned int nbCollisions;
  std::vector<CkcdObjectShPtr> objectVec1, objectVec2;

  if(getCollisionVec(nbCollisions, objectVec1, objectVec2) == true){
    std::cout<<" ------- "<<nbCollisions<<" collision(s) detected.-------- " <<std::endl;
    for(unsigned int i=0; i<nbCollisions; i++){

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec1[i]);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec2[i]);
      
      std::cout<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()<<std::endl;
    }
    std::cout<<" --------------------------"<<std::endl;
    return 0;
  }

  // std::cout<<std::endl<<" no collision detected. Closest objects: ";
    double dist;
    CkitPoint3 o_point1,  o_point2;
    CkcdObjectShPtr object1, object2;
      
    if(getExactDistance(dist, o_point1, o_point2, object1, object2) == KD_OK){

      return dist;
      /*
      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

      std::cout<<std::endl<<" kppPolyhedron "<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()
	       <<", distance "<<dist<<std::endl;
      std::cout<<" at ["<<o_point1[0]<<"] ["<<o_point1[1]<<"] ["<<o_point1[0]<<"] and ["
	       <<o_point2[0]<<"] ["<<o_point2[1]<<"] ["<<o_point2[0]<<"]"<<std::endl;
      */
      
    }
    else{
      std::cout<<"no outerlist registered for getExactDistance"<<std::endl;
      return -1;
    }
}


//=============================================================================

bool ChppBody::printCollisionStatus(const bool& detailInfoFlag )
{
  unsigned int nbCollisions;
  std::vector<CkcdObjectShPtr> objectVec1, objectVec2;

  if(getCollisionVec(nbCollisions, objectVec1, objectVec2) == true){
    std::cout<<" ------- "<<nbCollisions<<" collision(s) detected.-------- " <<std::endl;
    for(unsigned int i=0; i<nbCollisions; i++){

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec1[i]);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec2[i]);
      
      std::cout<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()<<std::endl;
    }
    std::cout<<" --------------------------"<<std::endl;
    return true;
  }

  if(detailInfoFlag){

    std::cout<<std::endl<<" no collision detected. Closest objects: ";
    double dist;
    CkitPoint3 o_point1,  o_point2;
    CkcdObjectShPtr object1, object2;
      
    if(getExactDistance(dist, o_point1, o_point2, object1, object2) == KD_OK){

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

      std::cout<<std::endl<<" kppPolyhedron "<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()
	       <<", distance "<<dist<<std::endl;
      std::cout<<" at ["<<o_point1[0]<<"] ["<<o_point1[1]<<"] ["<<o_point1[0]<<"] and ["
	       <<o_point2[0]<<"] ["<<o_point2[1]<<"] ["<<o_point2[0]<<"]"<<std::endl;
      
    }
    else{
      std::cout<<"no outerlist registered for getExactDistance"<<std::endl;
    }
  }
  return false;
}

//=============================================================================

void ChppBody::printCollisionStatusFast()
{
  unsigned int nbCollisions;
  CkcdObjectShPtr object1, object2;

  if(getCollision(nbCollisions, object1, object2) == true){
    // collision
    std::cout<<std::endl<<" ------- "<<nbCollisions<<" collision(s) detected.-------- " <<std::endl;

    getCollision(nbCollisions, object1, object2);

    CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
    CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

    std::cout<<"here"<<std::endl;
    std::cout<<kppPolyhedron1->name();
    std::cout<<" and "<<kppPolyhedron2->name()<<std::endl;
    std::cout<<" --------------------------"<<std::endl;
  }
  else{
    double dist;
    CkcdObjectShPtr object1, object2;
      
    if(getEstimatedDistance(dist, object1, object2) == KD_OK){

      std::cout<<std::endl<<" no collision detected. Closest objects: ";

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

      std::cout<<std::endl<<" kppPolyhedron "<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()
	       <<", distance "<<dist<<std::endl;
      
    }
  }
}


