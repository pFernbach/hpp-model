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

#ifndef HPP_MODEL_BODY_HH
#define HPP_MODEL_BODY_HH

/*************************************
INCLUDE
**************************************/

#include <KineoUtility/kitDefine.h>
#include <kcd2/kcdAnalysisType.h>
#include <kwsKcd2/kwsKCDBody.h>

#include "hpp/model/robot-dynamics-impl.hh"
#include "hpp/model/fwd.hh"

KIT_PREDEF_CLASS(CkppSolidComponentRef)
class CkitMat4;
class CkcdPoint;

namespace hpp {
  namespace model {

    /// \brief Bodies as geometric objects attached to a joint.

    /// It derives from KineoWorks CkwsKCDBody class and from an
    /// implementation of CjrlJoint.

    /// Objects attached to a body (called inner objects) are used for
    /// collision checking with selected objects of the environment
    /// (called outer objects).

    /// To attach an object to the body, call addInnerObject(). To
    /// select an object for collision checking with the body, call
    /// addOuterObject().

    /// Distances between pairs of inner objects and outer objects can
    /// also be computed. Setting <code>distanceComputation</code>
    /// to true in addInnerObject() or addOuterObject() specifies that
    /// distances should be computed for these objects. Each pair of
    /// such specified (inner, outer) objects gives rise to one
    /// distance computation when calling distAndPairsOfPoints(). The
    /// number of such pairs can be retrieved by calling
    /// nbDistPairs(). distAndPairsOfPoints() also returns distances
    /// and pairs of closest points for each computed pair.

    /// The constructor is protected and method create returns a
    /// shared pointer to the device.

    /// \sa Smart pointers documentation:
    /// http://www.boost.org/libs/smart_ptr/smart_ptr.htm
    class Body : public CkwsKCDBody
    {
    public:

      /// \brief Creation of a body
      /// \param name Name of the new body.
      /// \return A shared pointer to a new body.
      static BodyShPtr create(const std::string& name);

      /// \brief Get name of object.
      const std::string& name() {return name_;}

      /// \brief Set name of object.
      void name (const std::string& name) {name_ = name;}

      /// \name Define inner and outer objects
      /// @{
      ///
      ///
      /// \brief Add a geometric object to the body

      /// \param solidCompRef Reference to the solid component to add.
      /// \param position Position of the object before attaching it to the body
      /// (default value=Identity).
      /// \param distanceComputation whether this object should be put in the
      /// distance computation analysis.
      /// \return true if success, false otherwise.
      /// The object is added to the inner object list of the body.
      /// \note The body must be attached to a joint.
      bool addInnerObject(const CkppSolidComponentRefShPtr& solidCompRef,
			  const CkitMat4& position=CkitMat4(),
			  bool distanceComputation=false);

      /// \brief Add an object for collision testing with the body
      /// \param outerObject new object
      /// \param distanceComputation whether distance analyses should be added for
      /// this object.
      void addOuterObject(const CkcdObjectShPtr& outerObject,
			  bool distanceComputation=true);

      /// \brief Reset the list of outer objects
      void resetOuterObjects();

      ///
      /// @}
      ///

      /// \name Distance computation
      /// @{

      /// \brief Get the number of pairs of object for which distance is computed
      unsigned int nbDistPairs() { return distCompPairs_.size(); };

      /// \brief Compute exact distance and closest points between body and set of outer objects.

      /// \param pairId id of the pair of objects

      /// \retval outDistance Distance between body and outer objects
      /// \retval outPointBody Closest point on body (in global reference frame)
      /// \retval outPointEnv Closest point in outer object set (in global reference frame)
      ktStatus distAndPairsOfPoints(unsigned int pairId,
				    double& outDistance,
				    CkcdPoint& outPointBody,
				    CkcdPoint& outPointEnv);

      ///
      /// @}
      ///

    protected:

      /// \brief Pointer to the joint the body is attached to
      Joint* joint_;

      /// \brief Constructor by name.
      Body(std::string name);

      /// \brief Initialization of body
      /// \param weakPtr weak pointer to itself
      ktStatus init(const BodyWkPtr weakPtr);

    private:

      /// \brief Name of the body.
      std::string name_;

      /// \brief Inner objects for which distance computation is performed
      std::vector<CkcdObjectShPtr> innerObjForDist_;

      /// \brief Outer objects for which distance computation is performed
      std::vector<CkcdObjectShPtr> outerObjForDist_;

      /// \brief Collision analyses for this body
      /// Each pair (inner object, outer object) potentially defines an exact
      /// distance analysis. Only inner objects specified in attDistanceObjects
      /// define analyses.
      std::vector<CkcdAnalysisShPtr> distCompPairs_;

      /// \brief Weak pointer to itself
      BodyWkPtr weakPtr_;
    }; // class Body
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_BODY_HH
