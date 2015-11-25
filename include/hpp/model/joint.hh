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

#ifndef HPP_MODEL_JOINT_HH
# define HPP_MODEL_JOINT_HH

# include <cstddef>
# include <hpp/fcl/math/transform.h>
# include <hpp/model/config.hh>
# include <hpp/model/fwd.hh>

namespace hpp {
  namespace model {
    /// Robot joint
    ///
    /// A joint maps an input vector to a transformation of SE(3) from the
    /// parent frame to the joint frame.
    ///
    /// The input vector is provided through the configuration vector of the
    /// robot the joint belongs to. The joint input vector is composed of the
    /// components of the robot configuration starting at
    /// Joint::rankInConfiguration.
    ///
    /// The joint input vector represents a element of a Lie group, either
    /// \li a vector space for JointTranslation, and bounded JointRotation,
    /// \li the unit circle for non-bounded JointRotation joints,
    /// \li an element of SO(3) for JointSO3, represented by a unit quaternion.
    ///
    /// Operations specific to joints (uniform sampling of input space, straight
    /// interpolation, distance, ...) are performed by a JointConfiguration
    /// instance that has the same class hierarchy as Joint.
    class HPP_MODEL_DLLAPI Joint {
    public:
      /// \name Construction and copy and destruction
      /// \{

      /// Constructor
      /// \param initialPosition position of the joint before being inserted
      ///        in a kinematic chain,
      /// \param configSize dimension of the configuration vector,
      /// \param numberDof dimension of the velocity vector.
      Joint (const Transform3f& initialPosition, size_type configSize,
	     size_type numberDof);
      /// Copy constructor
      ///
      /// Clone body and therefore inner and outer objects (see Body::clone).
      Joint (const Joint& joint);

      /// Return pointer to copy of this
      ///
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const = 0;

      virtual ~Joint ();
      /// \}

      /// \name Name
      /// \{

      /// Set name
      virtual inline void name(const std::string& name)
      {
	name_ = name;
      }
      /// Get name
      virtual inline const std::string& name() const
      {
	return name_;
      }
      /// \}
      /// \name Position
      /// \{

      /// Joint initial position (when robot is in zero configuration)
      const Transform3f& initialPosition () const;
      /// Joint transformation
      const Transform3f& currentTransformation () const;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position) const = 0;
      /// Compute position of this joint and all its descendents.
      void recursiveComputePosition (ConfigurationIn_t configuration,
                     const Transform3f& parentPosition) const;

      /// Compute jacobian matrix of joint and all its descendents.
      void computeJacobian ();

      /// Get neutral configuration of joint
      ///
      /// Neutral configuration is
      /// \li 0 for translation joints,
      /// \li (1,0,0,0) for SO3 joints,
      /// \li (1,0) for unbounded rotation joint
      /// \li 0 for bounded rotation joint.
      vector_t neutralConfiguration () const
      {
	return neutralConfiguration_;
      }
      ///\}

      /// Return number of degrees of freedom
      const size_type& numberDof () const
      {
	return numberDof_;
      }
      /// Return number of degrees of freedom
      const size_type& configSize () const
      {
	return configSize_;
      }
      /// Return rank of the joint in the configuration vector
      const size_type& rankInConfiguration () const
      {
	return rankInConfiguration_;
      }
      /// Return rank of the joint in the velocity vector
      size_type rankInVelocity () const
      {
	return rankInVelocity_;
      }

      /// \name Kinematic chain
      /// \{

      /// Get a pointer to the parent joint (if any).
      JointPtr_t parentJoint () const
      {
	return parent_;
      }
      /// Add child joint
      /// \param joint child joint added to this one,
      /// \param computePositionInParent whether to compute position of the
      ///        child joint in this one's frame.
      ///
      /// \note When building a kinematic chain, we usually build the
      /// joint in its initial position and compute the (constant)
      /// position of the joint in its parent when adding the joint in
      /// the kinematic chain. When copying a kinematic chain, we copy the
      /// position of the joint in its parent frame and therefore we do
      /// not update it when adding the joint in the kinematic chain.
      void addChildJoint (JointPtr_t joint,
			  bool computePositionInParent = true);

      /// Number of child joints
      std::size_t numberChildJoints () const
      {
	return children_.size ();
      }
      /// Get child joint
      JointPtr_t childJoint (std::size_t rank) const
      {
	return children_ [rank];
      }
      /// Get position of joint in parent frame
      const Transform3f& positionInParentFrame () const
      {
	return positionInParentFrame_;
      }
      /// Set position of joint in parent frame
      void positionInParentFrame (const Transform3f& p)
      {
	positionInParentFrame_ = p;
      }
      ///\}

      /// \name Bounds
      /// \{
      /// Set whether given degree of freedom is bounded
      void isBounded (size_type rank, bool bounded);
      /// Get whether given degree of freedom is bounded
      bool isBounded (size_type rank) const;
      /// Get lower bound of given degree of freedom
      value_type lowerBound (size_type rank) const;
      /// Get upper bound of given degree of freedom
      value_type upperBound (size_type rank) const;
      /// Set lower bound of given degree of freedom
      void lowerBound (size_type rank, value_type lowerBound);
      /// Set upper bound of given degree of freedom
      void upperBound (size_type rank, value_type upperBound);
      /// Get upper bound on linear velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\mathbf {v}\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\mathbf{v}\f$ is the linear velocity of the joint frame.
      virtual value_type upperBoundLinearVelocity () const = 0;
      /// Get upper bound on angular velocity of the joint frame
      /// \return coefficient \f$\lambda\f$ such that
      /// \f{equation*}
      /// \forall \mathbf{q}_{joint}\ \ \ \ \ \ \|\omega\| \leq \lambda \|\mathbf{\dot{q}}_{joint}\|
      /// \f}
      /// where
      /// \li \f$\mathbf{q}_{joint}\f$ is any joint configuration,
      /// \li \f$\mathbf{\dot{q}}_{joint}\f$ is the joint velocity, and
      /// \li \f$\omega\f$ is the angular velocity of the joint frame.
      virtual value_type upperBoundAngularVelocity () const = 0;
      /// Maximal distance of joint origin to parent origin
      const value_type& maximalDistanceToParent () const
      {
	return maximalDistanceToParent_;
      }
      /// \}

      /// \name Jacobian
      /// \{

      /// Get const reference to Jacobian
      const JointJacobian_t& jacobian () const
      {
	return jacobian_;
      }
      /// Get non const reference to Jacobian
      JointJacobian_t& jacobian ()
      {
	return jacobian_;
      }
      /// \}
      /// Access to configuration space
      JointConfiguration* configuration () const {return configuration_;}
      /// Set robot owning the kinematic chain
      void robot (const DeviceWkPtr_t& device) {robot_ = device;}
      /// Access robot owning the object
      DeviceConstPtr_t robot () const { return robot_.lock ();}
      /// Access robot owning the object
      DevicePtr_t robot () { return robot_.lock ();}
      /// \name Body linked to the joint
      /// \{
      /// Get linked body
      BodyPtr_t linkedBody () const;
      /// Set linked body
      void setLinkedBody (const BodyPtr_t& body);
      /// \}

      /// \name Compatibility with urdf
      ///
      /// \{

      /// Get urdf link position in joint frame
      ///
      /// When parsing urdf models, joint frames are reoriented in order
      /// to rotate about their x-axis. For some applications, it is necessary
      /// to be able to recover the position of the urdf link attached to
      /// the joint.
      const Transform3f& linkInJointFrame () const
      {
	return linkInJointFrame_;
      }

      /// Set urdf link position in joint frame
      void linkInJointFrame (const Transform3f& transform)
      {
	linkInJointFrame_ = transform;
      }

      /// Get link name
      const std::string& linkName () const
      {
	return linkName_;
      }

      /// Set link name
      void linkName (const std::string& linkName)
      {
	linkName_ = linkName;
      }
      /// \}

      /// Display joint
      virtual std::ostream& display (std::ostream& os) const;
    protected:
      virtual void computeMaximalDistanceToParent () = 0;
      JointConfiguration* configuration_;
      mutable Transform3f currentTransformation_;
      Transform3f positionInParentFrame_;
      Transform3f linkInJointFrame_;
      mutable Transform3f T3f_;
      /// Mass of this and all descendants
      value_type mass_;
      /// Mass time center of mass of this and all descendants
      fcl::Vec3f massCom_;
      value_type maximalDistanceToParent_;
      vector_t neutralConfiguration_;
   private:
      /// Write a block of Jacobian
      ///
      /// child: joint the motion of which is generated by the degrees of
      ///      freedom of this
      ///
      /// child should be a descendant of this.
      /// This method writes in the jacobian of child the motion generated by
      /// this at the current position of child. If index is the rank of this
      /// in the velocity vector, the method fills colums from index to index +
      /// this joint number of degrees of freedom.
      virtual void writeSubJacobian (const JointPtr_t& child) = 0;
      /// Compute mass of this and all descendants
      value_type computeMass ();
      /// Compute the product m * com
      ///
      /// \li m is the mass of the joint and all descendants,
      /// \li com is the center of mass of the joint and all descendants.
      void computeMassTimesCenterOfMass ();
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const value_type& totalMass) = 0;
      size_type configSize_;
      size_type numberDof_;
      Transform3f initialPosition_;
      DeviceWkPtr_t robot_;
      BodyPtr_t body_;
      std::string name_;
      std::string linkName_;
      std::vector <JointPtr_t> children_;
      JointPtr_t parent_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      JointJacobian_t jacobian_;
      /// Rank of the joint in vector of children of parent joint.
      std::size_t rankInParent_;
      friend class Device;
      friend class ChildrenIterator;
      friend class CenterOfMassComputation;
    }; // class Joint

    /// Anchor Joint
    ///
    /// An anchor joint has no degree of freedom. It is usually used as an
    /// intermediate frame in a kinematic chain, or as a root joint for a
    /// multi-robot kinematic chain.
    class HPP_MODEL_DLLAPI JointAnchor : public Joint
    {
    public:
      JointAnchor (const Transform3f& initialPosition);
      JointAnchor (const JointAnchor& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const;
      virtual ~JointAnchor ();
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position) const;
      /// Get upper bound on linear velocity of the joint frame
      /// \return 0
      virtual value_type upperBoundLinearVelocity () const
      {
	return 0;
      }
      /// Get upper bound on angular velocity of the joint frame
      /// \return 0
      virtual value_type upperBoundAngularVelocity () const
      {
	return 0;
      }

    protected:
      virtual void computeMaximalDistanceToParent ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const value_type& totalMass);
    }; // class JointAnchor

    /// Spherical Joint
    ///
    /// map a unit quaternion as input vector to a rotation of SO(3).
    class HPP_MODEL_DLLAPI JointSO3 : public Joint
    {
    public:
      JointSO3 (const Transform3f& initialPosition);
      JointSO3 (const JointSO3& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position) const;
      virtual ~JointSO3 ();
      /// Get upper bound on linear velocity of the joint frame
      /// \return 0
      virtual value_type upperBoundLinearVelocity () const
      {
	return 0;
      }
      /// Get upper bound on angular velocity of the joint frame
      /// \return 1
      virtual value_type upperBoundAngularVelocity () const
      {
	return 1;
      }
    protected:
      virtual void computeMaximalDistanceToParent ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const value_type& totalMass);
      mutable fcl::Vec3f com_;
    }; // class JointSO3

    /// Rotation Joint
    ///
    /// Map an angle as one-dimensional input vector to a rotation around a
    /// fixed axis in parent frame.
    class HPP_MODEL_DLLAPI JointRotation : public Joint
    {
    public:
      /// Constructor
      /// \param initialPosition position of the joint in global frame before
      ///        being linked to a parent,
      /// \param configSize dimension of the configuration vector,
      /// \param numberDof dimension of the velocity vector.
      JointRotation (const Transform3f& initialPosition, size_type configSize,
		     size_type numberDof);
      JointRotation (const JointRotation& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const = 0;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position) const = 0;
      virtual ~JointRotation ();
      /// Get upper bound on linear velocity of the joint frame
      /// \return 0
      virtual value_type upperBoundLinearVelocity () const
      {
	return 0;
      }
      /// Get upper bound on angular velocity of the joint frame
      /// \return 1
      virtual value_type upperBoundAngularVelocity () const
      {
	return 1;
      }
    protected:
      virtual void computeMaximalDistanceToParent ();
      mutable fcl::Matrix3f R_;
      mutable fcl::Vec3f axis_;
      mutable fcl::Vec3f O2O1_;
      mutable fcl::Vec3f cross_;
      mutable fcl::Vec3f com_;
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const value_type& totalMass);
    }; // class JointRotation

    namespace jointRotation {
      /// Rotation about an axis without bound
      ///
      /// The configuration space of this joint is the unit circle, represented
      /// by \f$(q_0, q_1)\f$ such that \f$q_0^2 + q_1^2=1\f$
      class HPP_MODEL_DLLAPI UnBounded : public JointRotation
      {
      public:
	UnBounded (const Transform3f& initialPosition);
	UnBounded (const UnBounded& joint);
	/// Return pointer to copy of this
	/// Clone body and therefore inner and outer objects (see Body::clone).
	JointPtr_t clone () const;
	/// Compute position of joint
	/// \param configuration the configuration of the robot,
	/// \param parentPosition position of parent joint,
	/// \retval position position of this joint.
	virtual void computePosition (ConfigurationIn_t configuration,
				      const Transform3f& parentPosition,
				      Transform3f& position) const;

	virtual ~UnBounded ()
	{
	}
      private:
	mutable value_type cosAngle_;
	mutable value_type sinAngle_;
      }; // class UnBounded

      /// Rotation about an axis with bound
      ///
      /// The configuration space of this joint is an interval of angle.
      class HPP_MODEL_DLLAPI Bounded : public JointRotation
      {
      public:
	Bounded (const Transform3f& initialPosition);
	Bounded (const Bounded& joint);
	/// Return pointer to copy of this
	/// Clone body and therefore inner and outer objects (see Body::clone).
	JointPtr_t clone () const;
	/// Compute position of joint
	/// \param configuration the configuration of the robot,
	/// \param parentPosition position of parent joint,
	/// \retval position position of this joint.
	virtual void computePosition (ConfigurationIn_t configuration,
				      const Transform3f& parentPosition,
				      Transform3f& position) const;
	virtual ~Bounded ()
	{
	}
      private:
	mutable value_type angle_;
      }; // class Bounded
    } // namespace jointRotation
    /// Translation Joint
    ///
    /// Map a 1,2 or 3-dimensional input vector to a translation
    template <size_type dimension>
    class HPP_MODEL_DLLAPI JointTranslation : public Joint
    {
    public:
      JointTranslation (const Transform3f& initialPosition);
      JointTranslation (const JointTranslation <dimension>& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position) const;
      virtual ~JointTranslation ();
      /// Get upper bound on linear velocity of the joint frame
      /// \return 1
      virtual value_type upperBoundLinearVelocity () const
      {
	return 1;
      }
      /// Get upper bound on angular velocity of the joint frame
      /// \return 0
      virtual value_type upperBoundAngularVelocity () const
      {
	return 0;
      }
    protected:
      virtual void computeMaximalDistanceToParent ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const value_type& totalMass);
      mutable fcl::Vec3f t_;
      mutable fcl::Vec3f axis_ [dimension];
      mutable fcl::Vec3f com_;
    }; // class JointTranslation

    std::ostream& operator<< (std::ostream& os, const hpp::model::Joint& joint);
  } // namespace model
} // namespace hpp

namespace fcl {
  std::ostream& operator<< (std::ostream& os , const fcl::Transform3f& trans);
}
#endif // HPP_MODEL_JOINT_HH
