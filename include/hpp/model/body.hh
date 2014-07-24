//
// Copyright (c) 2013, 2014 CNRS
// Author: Florent Lamiraux
//
//
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
# define HPP_MODEL_BODY_HH

# include <hpp/util/pointer.hh>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace model {
    using fcl::Transform3f;

    /// Geometry associated to a Joint
    ///
    /// A body is a geometry container attached to a joint.
    /// The body contains objects (CollisionObject) that move with the joint and
    /// called <em>inner objects</em>.
    ///
    /// Collision and distance computation is performed against other objects
    /// that can be obstacles or objects attached to other joints. These object
    /// are called <em>outer objects</em> for the body.
    class HPP_MODEL_DLLAPI Body
    {
    public:
      /// \name Construction and copy and destruction
      /// @{
      /// Constructor
      Body ();
      /// Copy constructor
      Body (const Body& body);
      /// Clone body and attach to given joint
      ///
      /// inner and outer object lists are filled with copies of the objects
      /// contained in the lists of this. (See CollisionObject::clone).
      BodyPtr_t clone (const JointPtr_t& joint) const;
      virtual ~Body () {}
      /// @}

      /// \name Name
      /// \{

      /// Set name
      void name (const std::string& name) { name_ = name;}
      /// Get name
      std::string name () const { return name_;}
      /// \}

      /// Get joint holding the body
      JointPtr_t joint () const
      {
	return joint_;
      }
      /// Set joint holding the body
      void joint (JointPtr_t joint) {joint_ = joint;}

      /// \name Inner objects
      /// \{

      /// Add an object to the body
      /// \param object object to add. Position of object is expressed in
      /// world frame.
      /// \param collision whether this object should be considered for
      ///        collision
      /// \param distance  whether this object should be considered for
      ///        distance computation
      /// \note If object is already in body, do nothing.
      virtual void addInnerObject (const CollisionObjectPtr_t& object,
				   bool collision, bool distance);
      /// Remove an object to the body
      ///
      /// \param object object to remove
      /// \param collision whether this object should be removed from
      ///        list of collision objects
      /// \param distance  whether this object should be removed from
      ///        list of distance computation objects
      /// \note If object is not in body, do nothing
      virtual void removeInnerObject (const CollisionObjectPtr_t& object,
				      bool collision, bool distance);
      /// Access to inner objects
      /// \param type Collision or distance
      const ObjectVector_t& innerObjects (Request_t type) const;

      /// Get radius
      /// Radius is defined as an upper-bound to the distance of all points of
      /// the body to the origin of the joint that holds the body.
      value_type radius () const
      {
	return radius_;
      }
      /// \}

      /// \name Outer objects
      /// \{

      /// Add an object as obstacle for this body
      /// \param object object to add. Position of object is expressed in
      /// world frame.
      /// \param collision whether this object should be considered for
      ///        collision
      /// \param distance  whether this object should be considered for
      ///        distance computation
      /// \note If object is already in body, do nothing.
      /// \warning Added objects by this method will be unknown from the
      ///         Device and will not be reset by it
      virtual void addOuterObject (const CollisionObjectPtr_t& object,
				   bool collision, bool distance);
      /// Remove an obstacle to the body
      ///
      /// \param object object to remove
      /// \param collision whether this object should be removed from
      ///        list of collision objects
      /// \param distance  whether this object should be removed from
      ///        list of distance computation objects
      /// \note If object is not in body, do nothing
      virtual void removeOuterObject (const CollisionObjectPtr_t& object,
				      bool collision, bool distance);
      /// Access to outer objects
      /// \param type Collision or distance
      const ObjectVector_t& outerObjects (Request_t type) const;
      /// \}

      /// \name Collision and distance computation
      /// @{

      /// Test for collision
      /// \return true if collision, false if no collision
      bool collisionTest () const;

      /// Compute distances between pairs of objects stored in bodies
      void computeDistances (DistanceResults_t& results,
			     DistanceResults_t::size_type& offset);

      /// @}
      /// \name Inertial information
      /// @{
      /// Get position of center of mass in joint local reference frame.
      inline const fcl::Vec3f& localCenterOfMass () const
      {
	return localCom_;
      }

      /// Set postion of center of mass in joint reference frame.
      inline  void localCenterOfMass (const fcl::Vec3f& localCenterOfMass)
      {
	localCom_ = localCenterOfMass;
      }

      /// Get Intertia matrix expressed in joint local reference frame.
      inline  const matrix3_t& inertiaMatrix() const
      {
	return inertiaMatrix_;
      }

      /// Set inertia matrix.
      inline  void inertiaMatrix(const matrix3_t& inertiaMatrix)
      {
	inertiaMatrix_ = inertiaMatrix;
      }

      /// Get mass.
      inline  double mass() const
      {
	return mass_;
      }

      /// Set mass.
      inline  void mass(double mass)
      {
	mass_ = mass;
      }

      ///  @}
    private:
      void updateRadius (const CollisionObjectPtr_t& object);
      ObjectVector_t collisionInnerObjects_;
      ObjectVector_t collisionOuterObjects_;
      ObjectVector_t distanceInnerObjects_;
      ObjectVector_t distanceOuterObjects_;
      JointPtr_t joint_;
      std::string name_;
      /// Inertial information
      fcl::Vec3f localCom_;
      matrix3_t inertiaMatrix_;
      double mass_;
      value_type radius_;
    }; // class Body
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_BODY_HH
