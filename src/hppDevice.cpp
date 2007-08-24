/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux, Luis Delgado
 */

#include <iostream>
#include "hppModel/hppDevice.h" 
#include "hppModel/hppBody.h"

// ==========================================================================

ChppDevice::ChppDevice()
{
}

// ==========================================================================

ChppDevice::~ChppDevice()
{
}

// ==========================================================================

ChppDeviceShPtr ChppDevice::create(std::string inName)
{
  ChppDevice *hppDevice = new ChppDevice;
  ChppDeviceShPtr hppDeviceShPtr(hppDevice);

  if (hppDevice->init(hppDeviceShPtr, inName) != KD_OK) {
    hppDeviceShPtr.reset();
  }
  return hppDeviceShPtr;
}

// ==========================================================================

ChppDeviceShPtr ChppDevice::createCopy(const ChppDeviceShPtr& inDevice)
{
  ChppDevice* ptr = new ChppDevice(*inDevice);
  ChppDeviceShPtr deviceShPtr(ptr);

  if(KD_OK != ptr->init(deviceShPtr, inDevice))	{
    deviceShPtr.reset();
  }

  return deviceShPtr;
}

// ==========================================================================

CkwsDeviceShPtr ChppDevice::clone() const
{
  return ChppDevice::createCopy(attWeakPtr.lock());
}

// ==========================================================================

CkppComponentShPtr ChppDevice::cloneComponent() const
{
  return ChppDevice::createCopy(attWeakPtr.lock());
}

// ==========================================================================

bool ChppDevice::isComponentClonable() const
{
  return true;
}

// ==========================================================================

ktStatus ChppDevice::init(const ChppDeviceWkPtr& inDevWkPtr, const std::string &inName)
{
  ktStatus success = CkppDeviceComponent::init(inDevWkPtr, inName);

  if(KD_OK == success) {  
    attWeakPtr = inDevWkPtr;
  }
  return success;
}

// ==========================================================================

ktStatus ChppDevice::init(const ChppDeviceWkPtr& inWeakPtr, const ChppDeviceShPtr& inDevice)
{
  ktStatus  success = CkppDeviceComponent::init(inWeakPtr, inDevice);

  if(KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}

// ==========================================================================

ktStatus ChppDevice::axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
					     double& xMax, double& yMax, double& zMax) const
{

  TBodyVector bodyVector;
  this->getBodyVector(bodyVector);
  unsigned int j=bodyVector.size();
 
  xMin=9999999;
  yMin=9999999;
  zMin=9999999;
  xMax=-9999999;
  yMax=-9999999;
  zMax=-9999999;

  for(unsigned int i=0; i<j; i++)
  {
    /*Dynamic cast*/
    CkwsKCDBodyShPtr a;
    a=KIT_DYNAMIC_PTR_CAST(CkwsKCDBody, bodyVector[i]);
    if(!a)
    {
      std::cerr << "Error in axisAlignedBoundingBox, the CkwsBody not of subtype CkwsKCDBody" <<std::endl;
      return KD_ERROR;
    }
    computeBodyBoundingBox(a,xMin,yMin,zMin,xMax,yMax,zMax);
  }
  return KD_OK;
}

// ==========================================================================

ktStatus ChppDevice::ignoreDeviceForCollision (ChppDeviceShPtr inDevice ) {
   ktStatus status = KD_OK ;

  std::vector<CkcdObjectShPtr> inDeviceInnerObjectList;

  //
  // Get body vector of inDevice
  //
  CkwsDevice::TBodyVector inDeviceBodyVector;
  inDevice->getBodyVector(inDeviceBodyVector) ;
   
  //
  // For each body of inDevice,
  //
  for (unsigned int bodyId = 0 ;  bodyId < inDeviceBodyVector.size(); bodyId++)  {
    CkwsKCDBodyShPtr kcdBody;
    if (kcdBody = KIT_DYNAMIC_PTR_CAST(CkwsKCDBody, inDeviceBodyVector[bodyId])) {
      //
      // get the inner object list of the body
      //
      std::vector< CkcdObjectShPtr > kcdBodyInnerObjects = kcdBody->innerObjects() ;

      //
      // and put each inner object in inDeviceInnerObjectList
      //
      for (unsigned int objectId =0 ; objectId < kcdBodyInnerObjects.size() ; objectId ++) {
	CkcdObjectShPtr kcdObject = kcdBodyInnerObjects[objectId] ;
	inDeviceInnerObjectList.push_back(kcdObject) ;
        
      }
    }
    else {
      std::cerr << "ChppDevice::ignoreDeviceForCollision : body is not KCD body." << std::endl;
      return KD_ERROR ;
    } 
  }

  //
  // Set all inner objects of inDevice in the ignoredOuterObject list of each body of this device
  //
  CkwsDevice::TBodyVector thisBodyVector;
  getBodyVector(thisBodyVector);

  //
  // For each body of this device
  //
  for (unsigned int bodyId = 0 ;  bodyId < thisBodyVector.size(); bodyId++)  {
    CkwsKCDBodyShPtr thisKcdBody;
    if (thisKcdBody = KIT_DYNAMIC_PTR_CAST(CkwsKCDBody,thisBodyVector[bodyId])) {
      //
      // Get the list of ignored outer objects
      //
      std::vector<CkcdObjectShPtr> thisIgnoredObjectList = thisKcdBody->ignoredOuterObjects() ;

      //
      // Add to this list all the inner objects of inDevice
      //
      for( unsigned int objectId = 0 ; objectId < inDeviceInnerObjectList.size() ; objectId++) {
	thisIgnoredObjectList.push_back(inDeviceInnerObjectList[objectId]) ;
      }

      //
      // Set this new list as ignored outer object list of this device body
      //
      thisKcdBody->ignoredOuterObjects(thisIgnoredObjectList) ;
    }
    else {
      std::cerr << "ChppDevice::ignoreDeviceForCollision : body is not KCD body." << std::endl;
      return KD_ERROR ;
    } 
  }
  
  return status ;
}

// ==========================================================================

void ChppDevice::computeBodyBoundingBox(const CkwsKCDBodyShPtr& body, double& xMin, double& yMin, 
					double& zMin, double& xMax, double& yMax, double& zMax) const
{
  std::vector<CkcdObjectShPtr> listObject = body->innerObjects();
  unsigned int j= listObject.size();
  for(unsigned int i=0; i<j; i++)
  {
    ckcdObjectBoundingBox(listObject[i],xMin,yMin,zMin,xMax,yMax,zMax);
  }
}

// ==========================================================================

void ChppDevice::ckcdObjectBoundingBox(const CkcdObjectShPtr& object, double& xMin, double& yMin, 
				       double& zMin, double& xMax, double& yMax, double& zMax) const
{
  double x,y,z;

  object->boundingBox()->getHalfLengths(x, y, z) ;

  /*Matrices absolute et relative*/
  CkitMat4 matrixAbsolutePosition;
  CkitMat4 matrixRelativePosition;
  object->getAbsolutePosition(matrixAbsolutePosition);
  object->boundingBox()->getRelativePosition(matrixRelativePosition);

  /*Creer les points et change position points*/
  CkitMat4 matrixChangePosition = matrixAbsolutePosition*matrixRelativePosition;

  CkitPoint3 position[8];

  position[0]=matrixChangePosition*CkitPoint3( x, y, z);
  position[1]=matrixChangePosition*CkitPoint3( x, y,-z);
  position[2]=matrixChangePosition*CkitPoint3( x,-y, z);
  position[3]=matrixChangePosition*CkitPoint3(-x, y, z);
  position[4]=matrixChangePosition*CkitPoint3( x,-y,-z);
  position[5]=matrixChangePosition*CkitPoint3(-x,-y, z);
  position[6]=matrixChangePosition*CkitPoint3(-x, y,-z);
  position[7]=matrixChangePosition*CkitPoint3(-x,-y,-z);

  for(int i=0; i<8; i++)
  {
	if((position[i])[0]<xMin)
	{
	  xMin=(position[i])[0];
	}
	if((position[i])[1]<yMin)
	{
	  yMin=(position[i])[1];
	}
	if((position[i])[2]<zMin)
	{
	  zMin=(position[i])[2];
	}
	if((position[i])[0]>xMax)
	{
	  xMax=(position[i])[0];
	}
	if((position[i])[1]>yMax)
	{
	  yMax=(position[i])[1];
	}
	if((position[i])[2]>zMax)
	{
  	  zMax=(position[i])[2];
	}
  }

}

// ==========================================================================

ktStatus ChppDevice::addObstacle(const CkcdObjectShPtr& inObject)
{
  // Get robot vector of bodies.
  CkwsDevice::TBodyVector bodyVector;
  getBodyVector(bodyVector);
    
  // Loop over bodies of robot.
  for (CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++) {
    // Try to cast body into CkwsKCDBody
    CkwsKCDBodyShPtr kcdBody;
    ChppBodyShPtr hppBody;
    if (kcdBody = boost::dynamic_pointer_cast<CkwsKCDBody>(*bodyIter)) {
      std::vector< CkcdObjectShPtr > collisionList = kcdBody->outerObjects();
      collisionList.push_back(inObject);

      if(hppBody = boost::dynamic_pointer_cast<ChppBody>(kcdBody)){
	hppBody->setOuterObjects(collisionList);
      }
      else
	kcdBody->outerObjects(collisionList);

    }
    else {
      std::cout << "ChppDevice::addObstacle: body is not KCD body. Obstacle is not inserted." << std::endl;
    }
  }
  return KD_OK;
}


// ==========================================================================

void ChppDevice::setRootJoint(ChppJoint& inJoint)
{
  /*
    Set joint as Kineo root joint
  */
  CkppDeviceComponent::rootJointComponent(inJoint.kppJoint());

  /*
    Set joint as robotDynamics root joint
  */
  CimplDynamicRobot::rootJoint(*(inJoint.jrlJoint()));

  /*
    Store hppJoint in maps
  */
  attKppToHppJointMap[inJoint.kppJoint().get()] = &inJoint;
  attJrlToHppJointMap[inJoint.jrlJoint()] = &inJoint;
}

// ==========================================================================

ChppJoint* ChppDevice::getRootJoint() 
{
  /*
    Get CkppJointComponent root joint 
  */
  CkppJointComponentShPtr kppJointComponent = rootJointComponent();
    
  ChppJoint* joint = kppToHppJoint(kppJointComponent);
  return joint;
}

// ==========================================================================

void ChppDevice::registerJoint(ChppJoint& inHppJoint)
{
  /*
    Store hppJoint in maps
  */
  attKppToHppJointMap[inHppJoint.kppJoint().get()] = &inHppJoint;
  attJrlToHppJointMap[inHppJoint.jrlJoint()] = &inHppJoint;
}

// ==========================================================================

ChppJoint* ChppDevice::kppToHppJoint(CkppJointComponentShPtr inKppJoint)
{
  return attKppToHppJointMap[inKppJoint.get()];
}

// ==========================================================================

bool ChppDevice::hppSetCurrentConfig(const CkwsConfig& inConfig, EwhichPart inUpdateWhat)
{
  bool updateGeom = (inUpdateWhat == GEOMETRIC || inUpdateWhat == BOTH);
  bool updateDynamic = (inUpdateWhat == DYNAMIC || inUpdateWhat == BOTH);

  if (updateGeom) {
    if (CkppDeviceComponent::setCurrentConfig(inConfig) != KD_OK) {
      return false;
    }
  }
  if (updateDynamic) {
    unsigned int kwsConfigDim = inConfig.size();
    /* 
       Count the number of extra dofs of the CkppDeviceComponent
       since the first degrees of freedom of inConfig correspond to these extra-dofs.
    */
    unsigned int rankInCkwsConfig = countExtraDofs();
    std::vector< CkppJointComponentShPtr > kppJointVector;
    getJointComponentVector(kppJointVector);

    /*
      Allocate a vector for CjrldynamicRobot configuration
      The difference of size between KppDeviceComponent and CjrldynamicRobot is
      the number of extra dofs.
    */
    MAL_VECTOR_DIM(jrlConfig, double, kwsConfigDim-rankInCkwsConfig);

    /*
      Loop over CkppDeviceComponent joints
    */
    for (unsigned int iKppJoint=0; iKppJoint < kppJointVector.size(); iKppJoint++) {
      CkppJointComponentShPtr kppJoint = kppJointVector[iKppJoint];
      unsigned int jointDim = kppJoint->countDofComponents();
      /*
	Get associated CjrlJoint
      */
      CjrlJoint* jrlJoint = attKppToHppJointMap[kppJoint.get()]->jrlJoint();
      unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

      /*
	Check rank in configuration wrt  dimension.
      */
      if (jrlRankInConfig+jointDim > inConfig.size()) {
	std::cerr << "hppSetCurrentConfig: rank in configuration is more than configuration dimension." 
		  << std::endl;
	return false;
      }
      /*
	Fill dofs of this joint
      */
      for (unsigned int iJrlDof = jrlRankInConfig; iJrlDof < jrlRankInConfig+jointDim; iJrlDof++) {
	jrlConfig[iJrlDof] = inConfig.dofValue(rankInCkwsConfig);
	rankInCkwsConfig++;
      }
    }
    return CimplDynamicRobot::currentConfiguration(jrlConfig);
  }
  return false;
}

// ==========================================================================

bool ChppDevice::hppSetCurrentConfig(const vectorN& inConfig, EwhichPart inUpdateWhat)
{  
  bool updateGeom = (inUpdateWhat == GEOMETRIC || inUpdateWhat == BOTH);
  bool updateDynamic = (inUpdateWhat == DYNAMIC || inUpdateWhat == BOTH);

  if (updateDynamic) {
    if (!CimplDynamicRobot::currentConfiguration(inConfig)) {
      return false;
    }
  }
  if (updateGeom) {
    CkwsConfig kwsConfig(CkwsDeviceShPtr(this));
    unsigned int kwsConfigDim = kwsConfig.size();
    /* 
       Count the number of extra dofs of the CkppDeviceComponent and set them to 0
    */
    unsigned int rankInCkwsConfig;
    for (rankInCkwsConfig=0; rankInCkwsConfig < countExtraDofs(); rankInCkwsConfig++) {
      kwsConfig.dofValue(rankInCkwsConfig, 0);
    }

    std::vector< CkppJointComponentShPtr > kppJointVector;
    getJointComponentVector(kppJointVector);

    /*
      Loop over CkppDeviceComponent joints
    */
    for (unsigned int iKppJoint=0; iKppJoint < kppJointVector.size(); iKppJoint++) {
      CkppJointComponentShPtr kppJoint = kppJointVector[iKppJoint];
      unsigned int jointDim = kppJoint->countDofComponents();
      /*
	Get associated CjrlJoint
      */
      CjrlJoint* jrlJoint = attKppToHppJointMap[kppJoint.get()]->jrlJoint();
      unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

      /*
	Check rank in configuration wrt  dimension.
      */
      if (jrlRankInConfig > inConfig.size()) {
	std::cerr << "hppSetCurrentConfig: rank in configuration is more than configuration dimension." 
		  << std::endl;
	return false;
      }
      /*
	Fill dofs of this joint
      */
      for (unsigned int iJrlDof = jrlRankInConfig; iJrlDof < jrlRankInConfig+jointDim; iJrlDof++) {
	kwsConfig.dofValue(rankInCkwsConfig, inConfig[iJrlDof]);
	rankInCkwsConfig++;
      }
    }
    return (CkppDeviceComponent::setCurrentConfig(kwsConfig) == KD_OK);
  }
  return false;
}
