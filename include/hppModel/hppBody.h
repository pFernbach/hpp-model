/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPBODY_H_
#define HPPBODY_H_

/*************************************
INCLUDE
**************************************/

#include "KineoUtility/kitDefine.h"
#include "hppModel/hppImplRobotDynamics.h"
#include "kcd2/kcdAnalysisType.h"
#include "kwsKcd2/kwsKCDBody.h"

KIT_PREDEF_CLASS(ChppBody);
KIT_PREDEF_CLASS(CkcdObject);
KIT_PREDEF_CLASS(CkppSolidComponentRef);

class ChppJoint;
class CkitMat4;

/*************************************
CLASS
**************************************/
/**
 \brief This class represents bodies (geometric objects attached to a joint). 

 It derives from KineoWorks CkwsKCDBody class and from an implementation of 
 CjrlJoint.

 Objects attached to a body (called inner objects) are used for collision 
 checking with selected objects of the environment (called outer objects).

 To attach an object to the body, call addInnerObject(). To select an object
 for collision checking with the body, call addOuterObject().

 Distances between pairs of inner objects and outer objects can also
 be computed. Setting <code>inDistanceComputation</code> to true in
 addInnerObject() or addOuterObject() specifies that distances should
 be computed for these objects. Each pair of such specified (inner,
 outer) objects gives rise to one distance computation when calling
 distAndPairsOfPoints(). The number of such pairs can be retrieved by
 calling nbDistPairs(). distAndPairsOfPoints() also returns distances
 and pairs of closest points for each computed pair.
 
 The constructor is protected and method create returns a shared
 pointer to the device.

 \sa Smart pointers documentation:
 http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*/

class ChppBody : public CkwsKCDBody, public CimplBody
{
public:
  /**
     \brief Creation of a body
     \param inName Name of the new body.
     \return A shared pointer to a new body.
  */
  static ChppBodyShPtr create(std::string inName);

  /**
     \brief Get name of object.
  */
  const std::string& name() {return attName;};

  /**
     \name Define inner and outer objects
     @{
  */
  /**
     \brief Add a geometric object to the body
     
     \param inSolidComponentRef Reference to the solid component to add.
     \param inPosition Position of the object before attaching it to the body 
     (default value=Identity).
     \param inDistanceComputation whether this object should be put in the 
     distance computation analysis.

     \return true if success, false otherwise.

     The object is added to the inner object list of the body.

     \note The body must be attached to a joint.
  */
  bool addInnerObject(const CkppSolidComponentRefShPtr& inSolidComponentRef, 
		      const CkitMat4& inPosition=CkitMat4(),
		      bool inDistanceComputation=false);

  /**
     \brief Add an object for collision testing with the body

     \param inOuterObject new object
     \param inDistanceComputation whether distance analyses should be added for
     this object.
  */

  void 	addOuterObject(const CkcdObjectShPtr& inOuterObject, 
		       bool inDistanceComputation=true);

  /**
     \brief Reset the list of outer objects
  */
  void resetOuterObjects();

  /**
     @}
  */

  /**
     \name Distance computation
     @{
  */


  /**
     \brief Get the number of pairs of object for which distance is computed
  */
  inline unsigned int nbDistPairs() { return attDistCompPairs.size(); };

  /**
     \brief Compute exact distance and closest points between body and set of outer objects.

     \param inPairId id of the pair of objects
     \param inType Type of distance computation 
     (either CkcdAnalysisType::EXACT_DISTANCE or 
     CkcdAnalysisType::ESTIMATED_DISTANCE)

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body (in global reference frame)
     \retval outPointEnv Closest point in outer object set (in global reference frame)
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list
  */
  ktStatus distAndPairsOfPoints(unsigned int inPairId, 
				double& outDistance, 
				CkitPoint3& outPointBody, 
				CkitPoint3& outPointEnv,
				CkcdObjectShPtr &outObjectBody, 
				CkcdObjectShPtr &outObjectEnv, 
				CkcdAnalysisType::Type inType=
				CkcdAnalysisType::EXACT_DISTANCE);


  /**
     @}
  */

  /**
     \name Joint the body is attached to
     @{
  */

  /**
     \brief Store a pointer to the joint the body is attached to
  */
  void hppJoint(ChppJoint* inJoint) {attJoint = inJoint;};

  /**
     \brief Get a pointer to the joint the body is attached to
  */
  ChppJoint* hppJoint() const { return attJoint; };

  /**
     @}
  */

  /**
     \name Deprecated methods
     @{
  */

  /**
     \brief Add geometry to the body
     
     \param inSolidComponentRef Reference to the solid component to add.
     \param inPosition Position of the object before attaching it to the body 
     (default value=Identity).
     \param inDistanceComputation whether this object should be put in the 
     distance computation analysis.

     \return true if success, false otherwise.

     The object is added to the inner object list of the body.

     \note The body must be attached to a joint.

     \deprecated Call addInnerObject() instead
  */
  bool addSolidComponent(const CkppSolidComponentRefShPtr& inSolidComponentRef, 
			 const CkitMat4& inPosition=CkitMat4(),
			 bool inDistanceComputation=false)__attribute__ ((deprecated));

  /**
     \brief Attach objects to the body.
     \param inInnerObjects list of objects to attach to the body

     Previous objects if any are detached. 

     Objects are put in the left test tree of attExactAnalyzer for exact distance computation.

     \deprecated call addInnerObject() on each object of  inInnerObjects 
     instead.
  */

  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &inInnerObjects)__attribute__ ((deprecated));

  /**
     \brief Attach objects to the body in specified position
     \param inInnerObjects list of objects to attach to the body
     \param inPlacementVector Vector of homogeneous matrix specifying the position of each object in inInnerObjects.

     Previous objects if any are detached. 

     Objects are put in the left test tree of attExactAnalyzer for exact distance computation.

     \deprecated call addInnerObject() on each object of  inInnerObjects 
     instead.
  */

  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &inInnerObjects, 
			 const std::vector< CkitMat4 > &inPlacementVector)__attribute__ ((deprecated));


  /**
     \brief Defines the list of objects to be tested for collision with this body.
     \param inOuterObjects list of objects to be tested for collision for this body

     Previous objects if any are removed. 

     Objects are put in the right test tree of attExactAnalyzer for exact distance computation.

     \deprecated Call addOuterObject on each object of inOuterObjects instead
  */

  void 	setOuterObjects (const std::vector< CkcdObjectShPtr > &inOuterObjects)__attribute__ ((deprecated));

  /**
     \brief Compute exact distance and closest points between body and set of outer objects.

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body
     \retval outPointEnv Closest point in outer object set
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list
     
     \deprecated Call distAndPairsOfPoints() instead.
  */
  ktStatus getExactDistance(double& outDistance, CkitPoint3& outPointBody, CkitPoint3& outPointEnv,
			    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)__attribute__ ((deprecated));


  /**
     \brief Compute a lower bound of distance between body and set of outer objects.

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body
     \retval outPointEnv Closest point in outer object set
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list

     \deprecated Who does need this functions ?
  */
  ktStatus getEstimatedDistance(double &outDistance, 
				CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)__attribute__ ((deprecated));
   
  
  /**
     \brief Compute the set of inner and outer objects that are in collision with each other.

     \retval outNbCollision Number of pairs of objects in collision.
     \retval outObjectBodyVector Vector of objects in collision of the body.
     \retval outObjectEnvVector Vector of objects in collision in the outer object list.

     \return Whether there is a collision.

     \deprecated Who does need this function ?
  */
  bool getCollisionVec(unsigned int &outNbCollision, std::vector<CkcdObjectShPtr>& outObjectBodyVector, 
		       std::vector<CkcdObjectShPtr>& outObjectEnvVector)__attribute__ ((deprecated));

  /**
     \brief Compute collision and return the two first objects found in collision.

     \retval outNbCollision Number of pairs of object in collision found.
     \retval outObjectBody First object of the body found in collision.
     \retval outObjectEnv First object in the body outer list found in collision.

     \return Whether there is a collision.

     \deprecated What does need this function ?
  */
  bool getCollision(unsigned int& outNbCollision,
		    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv)__attribute__ ((deprecated));

  /**
    \deprecated What does this function do ?
  */
  bool printCollisionStatus(const bool& detailInfoFlag = false)__attribute__ ((deprecated));

  /**
    \deprecated What does this function do ?
  */
  void printCollisionStatusFast()__attribute__ ((deprecated));

  /**
     \brief Compute the minimum distance to the obstacle

     \return the minimum distance to the obstacle

     \deprecated Is this function really useful ?
  */
  double getMinDistance()__attribute__ ((deprecated));

  /**
     @}
  */

protected:
  
  /**
     \brief Pointer to the joint the body is attached to
  */
  ChppJoint* attJoint;

  /**
     \brief Constructor by name.
  */
  ChppBody(std::string inName) : attJoint(NULL), attName(inName) {};

  /**
     \brief Initialization of body

     \param inBodyWkPtr weak pointer to itself
  */
  ktStatus init(const ChppBodyWkPtr inBodyWkPtr);
  
private:

  /**
     \brief Name of the body.
  */
  std::string attName;

  /**
     \brief Collision analyser for this body
  */
  CkcdAnalysisShPtr attExactAnalyzer;

  /**
     \brief Set of inner objects for which distance computation is performed
  */
  std::vector<CkcdObjectShPtr> attInnerObjForDist;

  /**
     \brief Set of outer objects for which distance computation is performed
  */
  std::vector<CkcdObjectShPtr> attOuterObjForDist;

  /**
     \brief Collision analyses for this body

     Each pair (inner object, outer object) potentially defines an exact
     distance analysis. Only inner objects specified in attDistanceObjects
     define analyses.
  */
  std::vector<CkcdAnalysisShPtr> attDistCompPairs;

  /**
     \brief Weak pointer to itself
  */
  ChppBodyWkPtr attWeakPtr;
};


#endif /*HPPBODY_H_*/
