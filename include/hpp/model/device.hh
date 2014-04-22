///
/// Copyright (c) 2013, 2014 CNRS
/// Author: Florent Lamiraux
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

#ifndef HPP_MODEL_DEVICE_HH
#define HPP_MODEL_DEVICE_HH

# include <iostream>
# include <vector>

# include <hpp/model/fwd.hh>
# include <hpp/model/config.hh>
# include <hpp/model/distance-result.hh>
# include <hpp/model/object-iterator.hh>
# include <hpp/model/config.hh>

namespace hpp {
  namespace model {
    /// \brief Robot with geometric and dynamic model

    /// The creation of the device is done by Device::create(const
    /// std::string name).  This function returns a shared pointer
    /// to the newly created object.  \sa Smart pointers
    /// documentation:
    /// http://www.boost.org/libs/smart_ptr/smart_ptr.htm
    class HPP_MODEL_DLLAPI Device
    {
      friend class Body;
      friend class Joint;
    public:
      /// Flags to select computation
      /// To optimize computation time, computations performed by method
      /// computeForwardKinematics can be selected by calling method
      /// controlComputation.
      enum Computation_t {
	JACOBIAN = 0x1,
	VELOCITY = 0x2,
	ACCELERATION = 0x4,
	COM = 0x8,
	ALL = 0Xffff
      };
      /// \name Construction, copy and destruction
      /// \{
      virtual ~Device();

      /// \brief Clone as a CkwsDevice
      DevicePtr_t clone() const;

      /// \}

      /// Get name of device
      const std::string& name () {return name_;}

      /// \brief Creation of a new device
      /// \return a shared pointer to the new device
      /// \param name Name of the device (is passed to CkkpDeviceComponent)
      static DevicePtr_t create (std::string name);

      ///
      /// \brief Copy of a device
      /// \return A shared pointer to new device.
      /// \param device Device to be copied.
      static DevicePtr_t createCopy (const DevicePtr_t& device);

      /// \name Joints
      /// \{

      /// Set the root of the kinematic chain
      void rootJoint (JointPtr_t joint);

      /// Get root joint
      JointPtr_t rootJoint () const;

      /// Register joint in internal containers
      void registerJoint (JointPtr_t joint);

      /// Get vector of joints
      const JointVector_t& getJointVector () const;

      /// Get joint by name
      JointPtr_t getJointByName (const std::string& name);

      /// Size of configuration vectors
      const size_type& configSize () const
      {
	return configSize_;
      }

      /// Size of velocity vectors
      const size_type& numberDof () const
      {
	return numberDof_;
      }

      /// \}

      /// \name Current state
      /// \{

      /// Get current configuration
      const Configuration_t& currentConfiguration () const
      {
	return currentConfiguration_;
      }
      /// Set current configuration
      virtual void currentConfiguration (ConfigurationIn_t configuration)
      {
	if (configuration != currentConfiguration_) {
	  upToDate_ = false;
	  currentConfiguration_ = configuration;
	}
      }
      /// Get current velocity
      const vector_t& currentVelocity () const
      {
	return currentVelocity_;
      }

      /// Set current velocity
      void currentVelocity (vectorIn_t velocity)
      {
	upToDate_ = false;
	currentVelocity_ = velocity;
      }

      /// Get current acceleration
      const vector_t& currentAcceleration () const
      {
	return currentAcceleration_;
      }

      /// Set current acceleration
      void currentAcceleration (vectorIn_t acceleration)
      {
	upToDate_ = false;
	currentAcceleration_ = acceleration;
      }
      /// \}

      /// \name Mass and center of mass
      /// \{

      /// Get mass of robot
      const double& mass () const
      {
	return mass_;
      }
      /// Get position of center of mass
      const vector3_t& positionCenterOfMass () const
      {
	return com_;
      }
      /// Get Jacobian of center of mass with respect to configuration
      const ComJacobian_t& jacobianCenterOfMass () const
      {
	return jacobianCom_;
      }

      /// \}

      /// \name Collision and distance computation
      /// \{

      /// Add an object for collision or distance computation
      ///
      /// \param object object to add. Position of object is expressed in
      /// world frame.
      /// \param collision whether this object should be considered for
      ///        collision
      /// \param distance  whether this object should be considered for
      ///        distance computation
      virtual void addOuterObject (const CollisionObjectPtr_t &object,
				   bool collision, bool distance);

      /// Remove an object for collision or distance computation
      ///
      /// \param object object to remove
      /// \param collision whether this object should be removed from
      ///        list of collision objects
      /// \param distance  whether this object should be removed from
      ///        list of distance computation objects
      /// \note If object is not in body, do nothing
      virtual void removeOuterObject (const CollisionObjectPtr_t& object,
				      bool collision, bool distance);
				      
	  /// Reset all objects for collision or distance computation from a device		      
	  void resetOuterObject (bool collision, bool distance);

      /// Add collision pairs between objects attached to two joints
      ///
      /// \param joint1 first joint
      /// \param joint2 second joint
      /// \param type collision or distance.
      ///
      /// Define collision pair between each object of joint 1 body and
      /// each object of joint2 body.
      virtual void addCollisionPairs (const JointPtr_t& joint1,
				      const JointPtr_t& joint2,
				      Request_t type);

      /// Get the set of collision pairs
      ///
      /// \return pairs of joints for which method addCollisionPairs has been
      ///         called.
      const InteractionPairs_t& collisionPairs (Request_t type) const;

      /// Iterator over inner objects of the device
      /// \param type Collision or distance
      ObjectIterator objectIterator (Request_t type);
      /// Iterator end
      ObjectIterator objectIteratorEnd (Request_t type);
      
      /// Mapping of outer objects used in collision checking using the joint name
      const ObjectVector_t& outerCollisionObjectVectors (const std::string& name);
            
      /// Mapping of outer objects used in distance computation using the joint name
      const ObjectVector_t& outerDistanceObjectVectors (const std::string& name);

      /// Test collision of current configuration
      /// \warning Users should call computeForwardKinematics first.
      bool collisionTest () const;

      /// Compute distances between pairs of objects stored in bodies
      void computeDistances ();

      /// Get result of distance computations
      const DistanceResults_t&
	distanceResults () const {return distances_;};
      /// \}

      /// \name Forward kinematics
      /// \{

      /// Select computation
      /// Optimize computation time by selecting only necessary values in
      /// method computeForwardKinematics.
      void controlComputation (const Computation_t& flag)
      {
	computationFlag_ = flag;
	upToDate_ = false;
      }
      /// Get computation flag
      Computation_t computationFlag () const
      {
	return computationFlag_;
      }
       /// Compute forward kinematics
      virtual void computeForwardKinematics ();
      /// \}
    protected:
      /// \brief Constructor
      Device(const std::string& name);

      ///
      /// \brief Initialization.
      ///
      void init(const DeviceWkPtr& weakPtr);

    private:
      /// Recompute number of distance pairs
      void updateDistances ();
      void computeJointPositions ();
      void computeJointJacobians ();
      void computeMass ();
      void computePositionCenterOfMass ();
      void computeJacobianCenterOfMass ();
      void resizeJacobians ();
      std::string name_;
      DistanceResults_t distances_;
      InteractionPairs_t collisionPairs_;
      InteractionPairs_t distancePairs_;
      JointByName_t jointByName_;
      JointVector_t jointVector_;
      JointPtr_t rootJoint_;
      size_type numberDof_;
      size_type configSize_;
      Configuration_t currentConfiguration_;
      vector_t currentVelocity_;
      vector_t currentAcceleration_;
      vector3_t com_;
      ComJacobian_t jacobianCom_;
      double mass_;
      bool upToDate_;
      Computation_t computationFlag_;
      DeviceWkPtr weakPtr_;
      
      /// Map of outer collision objects by joints names
      std::map <std::string, ObjectVector_t> outerCollisionObjectVectorsMap_;
      /// Map of outer distance objects by joints names
      std::map <std::string, ObjectVector_t> outerDistanceObjectVectorsMap_;  // ObjectVector_t distanceObstacles_;   CollisionObjectPtr_t
    }; // class Device
  } // namespace model
} // namespace hpp

std::ostream& operator<<(std::ostream& os, hpp::model::Device& device);

#endif // HPP_MODEL_DEVICE_HH
