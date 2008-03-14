/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux, Luis Delgado
 */

#include <iostream>

#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoModel/kppRotationJointComponent.h"
#include "KineoModel/kppTranslationJointComponent.h"

#include "hppModel/hppDevice.h" 
#include "hppModel/hppBody.h"
#include "kwsioConfig.h"

#define ODEBUG(x)
//#define ODEBUG(x) std::cerr << "hppDevice :" << x << std::endl

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

void ChppDevice::setRootJoint(ChppJoint* inJoint)
{
  /*
    Set joint as Kineo root joint
  */
  CkppDeviceComponent::rootJointComponent(inJoint->kppJoint());

  /*
    Set joint as robotDynamics root joint
  */
  CimplDynamicRobot::rootJoint(*(inJoint->jrlJoint()));
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

void ChppDevice::registerJoint(ChppJoint* inHppJoint)
{
  /*
    Store hppJoint in maps
  */
  attKppToHppJointMap[inHppJoint->kppJoint()] = inHppJoint;
  attJrlToHppJointMap[inHppJoint->jrlJoint()] = inHppJoint;
}

// ==========================================================================

ChppJoint* ChppDevice::kppToHppJoint(CkppJointComponentShPtr inKppJoint)
{
  if (attKppToHppJointMap.count(inKppJoint) != 1) {
    std::cerr << "ChppDevice::kppToHppJoint: no ChppJoint for this CkppJointComponent." << std::endl;
    return NULL;
  }
  return attKppToHppJointMap[inKppJoint];
}

// ==========================================================================

bool ChppDevice::hppSetCurrentConfig(const CkwsConfig& inConfig, EwhichPart inUpdateWhat)
{
  bool updateGeom = (inUpdateWhat == GEOMETRIC || inUpdateWhat == BOTH);
  bool updateDynamic = (inUpdateWhat == DYNAMIC || inUpdateWhat == BOTH);

  if (updateGeom) {
    ODEBUG("hppSetCurrentConfig: updating geometric part");
    ODEBUG("hppSetCurrentConfig: inConfig = " << inConfig);


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
    // MAL_VECTOR_DIM(jrlConfig, double, kwsConfigDim-rankInCkwsConfig);
    MAL_VECTOR_DIM(jrlConfig, double, numberDof());

    /*
      Loop over CkppDeviceComponent joints
    */
    for (unsigned int iKppJoint=0; iKppJoint < kppJointVector.size(); iKppJoint++) {
      CkppJointComponentShPtr kppJoint = kppJointVector[iKppJoint];
      unsigned int jointDim = kppJoint->countDofComponents();

      /* 
	 Check if the joint is found in the associated map
      */
      if(attKppToHppJointMap[kppJoint] == NULL){
	// std::cout<<"joint "<<kppJoint->name()<<" "<<"... not found. "<<endl;
	continue;
      }

      /*
	Get associated CjrlJoint
      */
      CjrlJoint* jrlJoint = attKppToHppJointMap[kppJoint]->jrlJoint();
      unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

      ODEBUG("iKppJoint=" << kppJoint->name() << " jrlRankInConfig=" << jrlRankInConfig);

      /*
	Check rank in configuration wrt  dimension.
      */
      if (jrlRankInConfig+jointDim > inConfig.size()) {
	std::cerr << "hppSetCurrentConfig: rank in configuration is more than configuration dimension(rank = " << jrlRankInConfig << ", dof = " << jointDim 
		  << ")." << std::endl;
	std::cout << "vectorN: " << jrlConfig << std::endl;
	return false;
      }
      /*
	Cast joint into one of the possible types
      */
      if (CkppFreeFlyerJointComponentShPtr jointFF = KIT_DYNAMIC_PTR_CAST(CkppFreeFlyerJointComponent,
									  kppJoint)) {
	// Translations along x, y, z
	jrlConfig[jrlRankInConfig] = inConfig.dofValue(rankInCkwsConfig);
	jrlConfig[jrlRankInConfig+1] = inConfig.dofValue(rankInCkwsConfig+1);
	jrlConfig[jrlRankInConfig+2] = inConfig.dofValue(rankInCkwsConfig+2);
	double rx = inConfig.dofValue(rankInCkwsConfig+3);
	double ry = inConfig.dofValue(rankInCkwsConfig+4);
	double rz = inConfig.dofValue(rankInCkwsConfig+5);
	// Convert KineoWorks rotation angles to roll, pitch, yaw
	double roll, pitch, yaw;
	YawPitchRollToRollPitchYaw(rx, ry, rz, roll, pitch, yaw);
	jrlConfig[jrlRankInConfig+3] = roll;
	jrlConfig[jrlRankInConfig+4] = pitch;
	jrlConfig[jrlRankInConfig+5] = yaw;
	ODEBUG("Joint value: " << jrlConfig[jrlRankInConfig] << ", "
	       << jrlConfig[jrlRankInConfig+1] << ", "
	       << jrlConfig[jrlRankInConfig+2] << ", "
	       << jrlConfig[jrlRankInConfig+3] << ", "
	       << jrlConfig[jrlRankInConfig+4] << ", "
	       << jrlConfig[jrlRankInConfig+5]);
	rankInCkwsConfig += 6;
      }
      else if(CkppRotationJointComponentShPtr jointRot = KIT_DYNAMIC_PTR_CAST(CkppRotationJointComponent,
									      kppJoint)) {
	jrlConfig[jrlRankInConfig] = inConfig.dofValue(rankInCkwsConfig);
	ODEBUG("Joint value: " << jrlConfig[jrlRankInConfig]);
	rankInCkwsConfig ++;
      }
      else if(CkppTranslationJointComponentShPtr jointTrans = KIT_DYNAMIC_PTR_CAST(CkppTranslationJointComponent,
										   kppJoint)) {
	jrlConfig[jrlRankInConfig] = inConfig.dofValue(rankInCkwsConfig);
	ODEBUG("Joint value: " << jrlConfig[jrlRankInConfig]);
	rankInCkwsConfig ++;
      }
      else {
	std::cerr << "hppSetCurrentConfig: unknown type of joint." 
		  << std::endl;
	std::cout << "vectorN: " << jrlConfig << std::endl;
	return false;
      }
    }

    ODEBUG("hppSetCurrentConfig: jrlConfig = " << jrlConfig);

    if (!currentConfiguration(jrlConfig)) {
      return false;
    }
    if (!computeForwardKinematics()) {
	return false;
    }
  }
  return true;
}

// ==========================================================================

bool ChppDevice::hppSetCurrentConfig(const vectorN& inConfig, EwhichPart inUpdateWhat)
{  
  bool updateGeom = (inUpdateWhat == GEOMETRIC || inUpdateWhat == BOTH);
  bool updateDynamic = (inUpdateWhat == DYNAMIC || inUpdateWhat == BOTH);

  if (updateDynamic) {
    if (!currentConfiguration(inConfig)) {
      return false;
    }
    if (!computeForwardKinematics()) {
	return false;
    }
  }
  if (updateGeom) {
    CkwsConfig kwsConfig(CkwsDeviceShPtr(this));
    unsigned int kwsConfigDim = kwsConfig.size();
    /* 
       Count the number of extra dofs of the CkppDeviceComponent.
    */
    unsigned int rankInCkwsConfig = countExtraDofs();

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
      CjrlJoint* jrlJoint = attKppToHppJointMap[kppJoint]->jrlJoint();
      unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

      /*
	Check rank in configuration wrt  dimension.
      */
      if (jrlRankInConfig > inConfig.size()) {
	std::cerr << "hppSetCurrentConfig: rank in configuration is more than configuration dimension(" << jrlRankInConfig << ")." 
		  << std::endl;
	return false;
      }
      /*
	Cast joint into one of the possible types
      */
      if (CkppFreeFlyerJointComponentShPtr jointFF = KIT_DYNAMIC_PTR_CAST(CkppFreeFlyerJointComponent,
									  kppJoint)) {
	// Translations along x, y, z
	kwsConfig.dofValue(rankInCkwsConfig, inConfig[jrlRankInConfig]);
	kwsConfig.dofValue(rankInCkwsConfig+1, inConfig[jrlRankInConfig+1]);
	kwsConfig.dofValue(rankInCkwsConfig+2, inConfig[jrlRankInConfig+2]);

	double roll = inConfig[jrlRankInConfig+3];
	double pitch = inConfig[jrlRankInConfig+4];
	double yaw = inConfig[jrlRankInConfig+5];

	// Convert KineoWorks rotation angles to roll, pitch, yaw
	double rx, ry, rz;
	RollPitchYawToYawPitchRoll(roll, pitch, yaw, rx, ry, rz);

	kwsConfig.dofValue(rankInCkwsConfig+3, rx);
	kwsConfig.dofValue(rankInCkwsConfig+4, ry);
	kwsConfig.dofValue(rankInCkwsConfig+5, rz);
	
	rankInCkwsConfig+= 6;
      }
      else if(CkppRotationJointComponentShPtr jointRot = KIT_DYNAMIC_PTR_CAST(CkppRotationJointComponent,
									      kppJoint)) {
	kwsConfig.dofValue(rankInCkwsConfig, inConfig[jrlRankInConfig]);
	rankInCkwsConfig++;
      }
      else if(CkppTranslationJointComponentShPtr jointTrans = KIT_DYNAMIC_PTR_CAST(CkppTranslationJointComponent,
										   kppJoint)) {
	kwsConfig.dofValue(rankInCkwsConfig, inConfig[jrlRankInConfig]);
	rankInCkwsConfig++;
      }
      else {
	std::cerr << "hppSetCurrentConfig: unknown type of joint." 
		  << std::endl;
	std::cout << "vectorN: " << inConfig << std::endl;
	return false;
      }
    }
    return (CkppDeviceComponent::setCurrentConfig(kwsConfig) == KD_OK);
  }
  return true;
}

void ChppDevice::RollPitchYawToYawPitchRoll(const double& inRx, const double& inRy, const double& inRz,
					    double& outRx, double& outRy, double& outRz)
{
  const double cRx=cos(inRx),sRx=sin(inRx),cRy=cos(inRy),sRy=sin(inRy),cRz=cos(inRz),sRz=sin(inRz);
  double R[3][3];
  R[0][0] = cRy*cRz;
  R[0][1] = sRx*sRy*cRz - cRx*sRz;
  R[0][2] = cRx*sRy*cRz + sRx*sRz;
  R[1][0] = cRy*sRz;
  R[1][1] = sRx*sRy*sRz + cRx*cRz;
  R[1][2] = cRx*sRy*sRz - sRx*cRz;
  //R[2][0] = -sRy;
  //R[2][1] = sRx*cRy;
  R[2][2] = cRx*cRy;
  
  // make sure all values are in [-1,1]
  // as trigonometric functions would
  // fail when given values such as 1.00000001
  for(unsigned int i=0; i<3; i++) {
    for(unsigned int j=0; j<3; j++) {
      if(R[i][j] < -1.) {
	R[i][j] = -1;
      }
      else if(R[i][j] > 1.) {
	R[i][j] = 1;
      }
    }
  }
  
  outRy = asin(R[0][2]);
  double cosOutRy = cos(outRy);

  double sinOutRz, cosOutRz;

  if(fabs(cosOutRy) > 1e-6) {
    double cosOutRx =  R[2][2] / cosOutRy;
    double sinOutRx = -R[1][2] / cosOutRy;
    
    outRx = atan2(sinOutRx, cosOutRx);
    
    double cosOutRz =  R[0][0] / cosOutRy;
    double sinOutRz = -R[0][1] / cosOutRy;
    outRz = atan2(sinOutRz, cosOutRz);
  }
  else {
    outRx = 0.;
    
    double cosOutRz = R[1][1];
    double sinOutRz = R[1][0];
    outRz = atan2(sinOutRz, cosOutRz);
  }
}

void ChppDevice::YawPitchRollToRollPitchYaw(const double& inRx, const double& inRy, const double& inRz,
					    double& outRx, double& outRy, double& outRz)
{
  const double cRx=cos(inRx),sRx=sin(inRx),cRy=cos(inRy),sRy=sin(inRy),cRz=cos(inRz),sRz=sin(inRz);
  double R[3][3];
  R[0][0] = cRz*cRy;
  R[0][1] = -sRz*cRy;
  //R[0][2] = sRy;
  R[1][0] = cRz*sRy*sRx+sRz*cRx;
  R[1][1] = cRz*cRx-sRz*sRy*sRx;
  //R[1][2] = -cRy*sRx;
  R[2][0] = sRz*sRx-cRz*sRy*cRx;
  R[2][1] = sRz*sRy*cRx+cRz*sRx;
  R[2][2] = cRx*cRy;

  // make sure all values are in [-1,1]
  // as trigonometric functions would
  // fail when given values such as 1.00000001
  for(unsigned int i=0; i<3; i++) {
    for(unsigned int j=0; j<3; j++) {
      if(R[i][j] < -1.) {
	R[i][j] = -1;
      }
      else if(R[i][j] > 1.) {
	R[i][j] = 1;
      }
    }
  }
  outRy = -asin(R[2][0]);
  double cosOutRy = cos(outRy);

  if (fabs(cosOutRy) > 1e-6) {
    double sinOutRx = R[2][1]/cosOutRy;
    double cosOutRx = R[2][2]/cosOutRy;
    outRx = atan2(sinOutRx, cosOutRx);

    double cosOutRz = R[0][0]/cosOutRy;
    double sinOutRz = R[1][0]/cosOutRy;
    outRz = atan2(sinOutRz, cosOutRz);
  }
  else {
    outRx = 0;

    double cosOutRz = R[1][1];
    double sinOutRz = -R[0][1];
    outRz = atan2(sinOutRz, cosOutRz);
  }
}

// ==========================================================================

template <class CkppJnt, class CjrlJnt> ChppJoint* ChppDevice::createJoint(std::string inName, 
									   const CkitMat4& inInitialPosition)
{
  /*
    Create kppJointComponent
  */
  CkppJointComponentShPtr kppJoint = CkppJnt::create(inName);
  if (kppJoint) {
    kppJoint->kwsJoint()->setCurrentPosition(inInitialPosition);
  } else {
    return NULL;
  }

  /*
    Convert homogeneous matrix to abstract matrix type matrix4d
  */
  matrix4d initialPos = ChppJoint::abstractMatrixFromCkitMat4(inInitialPosition);
  CjrlJoint* jrlJoint = new CjrlJnt(initialPos);
  if (!jrlJoint) {
    delete jrlJoint;
    return NULL;
  }

  /*
    Create ChppJoint
  */
  ChppJoint* hppJoint = new ChppJoint(kppJoint, jrlJoint, attWeakPtr);

  registerJoint(hppJoint);
  return hppJoint;
}

// ==========================================================================

ChppJoint* ChppDevice::createFreeFlyer(std::string inName, const CkitMat4& inInitialPosition)
{
  ChppJoint* hppJoint = createJoint<CkppFreeFlyerJointComponent, CimplJointFreeFlyer>(inName, inInitialPosition);

  return hppJoint;
}

// ==========================================================================

ChppJoint* ChppDevice::createRotation(std::string inName, const CkitMat4& inInitialPosition)
{
  return createJoint<CkppRotationJointComponent, CimplJointRotation>(inName, inInitialPosition);
}

// ==========================================================================

ChppJoint* ChppDevice::createTranslation(std::string inName, const CkitMat4& inInitialPosition)
{
  return createJoint<CkppTranslationJointComponent, CimplJointTranslation>(inName, inInitialPosition);
}

/**
  \brief Write a device in a stream.
*/
std::ostream& operator<<(std::ostream& os, ChppDevice& inHppDevice)
{
  os << "Device: " << inHppDevice.name() << std::endl;
  os << std::endl;
  os << "  Current configuration: " << inHppDevice.currentConfiguration() << std::endl;
  os << std::endl;
  os << std::endl;
  os << "  Writing kinematic chain" << std::endl;

  //
  // Go through joints and output each joint
  //
  ChppJoint* hppJoint = inHppDevice.getRootJoint();

  if (hppJoint) {
    os << *hppJoint << std::endl;
  }
  // Get position of center of mass
  MAL_S3_VECTOR(com, double);
  com = inHppDevice.positionCenterOfMass();
  
  //debug
  cout<<"total mass "<<inHppDevice.mass() <<", COM: "<< MAL_S3_VECTOR_ACCESS(com, 0) 
      <<", "<< MAL_S3_VECTOR_ACCESS(com, 1) 
      <<", "<< MAL_S3_VECTOR_ACCESS(com, 2)
      <<endl;


  return os;
}
