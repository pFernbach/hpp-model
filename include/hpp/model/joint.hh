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
# include <fcl/math/transform.h>
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
				    Transform3f& position) = 0;

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
      ///\}

      /// \name Bounds
      /// \{
      /// Set whether given degree of freedom is bounded
      void isBounded (size_type rank, bool bounded);
      /// Get whether given degree of freedom is bounded
      bool isBounded (size_type rank) const;
      /// Get lower bound of given degree of freedom
      double lowerBound (size_type rank) const;
      /// Get upper bound of given degree of freedom
      double upperBound (size_type rank) const;
      /// Set lower bound of given degree of freedom
      void lowerBound (size_type rank, double lowerBound);
      /// Set upper bound of given degree of freedom
      void upperBound (size_type rank, double upperBound);
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

      /// Display joint
      virtual std::ostream& display (std::ostream& os) const;
    protected:
      JointConfiguration* configuration_;
      Transform3f currentTransformation_;
      Transform3f positionInParentFrame_;
      Transform3f T3f_;
      /// Mass of this and all descendants
      double mass_;
      /// Mass time center of mass of this and all descendants
      fcl::Vec3f massCom_;
   private:
      /// Compute position of this joint and all its descendents.
      void recursiveComputePosition (ConfigurationIn_t configuration,
				     const Transform3f& parentPosition);

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
      void computeJacobian ();
      /// Compute mass of this and all descendants
      double computeMass ();
      /// Compute the product m * com
      ///
      /// \li m is the mass of the joint and all descendants,
      /// \li com is the center of mass of the joint and all descendants.
      void computeMassTimesCenterOfMass ();
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass) = 0;
      size_type configSize_;
      size_type numberDof_;
      Transform3f initialPosition_;
      DeviceWkPtr_t robot_;
      BodyPtr_t body_;
      std::string name_;
      std::vector <JointPtr_t> children_;
      JointPtr_t parent_;
      size_type rankInConfiguration_;
      size_type rankInVelocity_;
      JointJacobian_t jacobian_;
      /// Rank of the joint in vector of children of parent joint.
      std::size_t rankInParent_;
      friend class Device;
      friend class ChildrenIterator;
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
				    Transform3f& position);
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass);
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
				    Transform3f& position);
      virtual ~JointSO3 ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass);
      mutable fcl::Vec3f com_;
    }; // class JointSO3

    /// Rotation Joint
    ///
    /// Map an angle as one-dimensional input vector to a rotation around a
    /// fixed axis in parent frame.
    class HPP_MODEL_DLLAPI JointRotation : public Joint
    {
    public:
      JointRotation (const Transform3f& initialPosition);
      JointRotation (const JointRotation& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position);
      virtual ~JointRotation ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass);
      fcl::Matrix3f R_;
      mutable double angle_;
      mutable fcl::Vec3f axis_;
      mutable fcl::Vec3f O2O1_;
      mutable fcl::Vec3f cross_;
      mutable fcl::Vec3f com_;
    }; // class JointRotation

    /// Translation Joint
    ///
    /// Map a length as one-dimensional input vector to a translation along a
    /// fixed axis in parent frame.
    class HPP_MODEL_DLLAPI JointTranslation : public Joint
    {
    public:
      JointTranslation (const Transform3f& initialPosition);
      JointTranslation (const JointTranslation& joint);
      /// Return pointer to copy of this
      /// Clone body and therefore inner and outer objects (see Body::clone).
      virtual JointPtr_t clone () const;
      /// Compute position of joint
      /// \param configuration the configuration of the robot,
      /// \param parentPosition position of parent joint,
      /// \retval position position of this joint.
      virtual void computePosition (ConfigurationIn_t configuration,
				    const Transform3f& parentPosition,
				    Transform3f& position);
      virtual ~JointTranslation ();
    private:
      virtual void writeSubJacobian (const JointPtr_t& child);
      virtual void writeComSubjacobian (ComJacobian_t& jacobian,
					const double& totalMass);
      fcl::Vec3f t_;
      mutable fcl::Vec3f axis_;
      mutable fcl::Vec3f com_;
    }; // class JointTranslation

  } // namespace model
} // namespace hpp

std::ostream& operator<< (std::ostream& os , const fcl::Transform3f& trans);
std::ostream& operator<< (std::ostream& os, const hpp::model::Joint& joint);

#endif // HPP_MODEL_JOINT_HH
