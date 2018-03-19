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

#ifndef HPP_MODEL_DEVICE_HH
#define HPP_MODEL_DEVICE_HH

# include <iostream>
# include <vector>
# include <list>

# include <hpp/util/debug.hh>
# include <hpp/model/fwd.hh>
# include <hpp/model/config.hh>
# include <hpp/model/distance-result.hh>
# include <hpp/model/extra-config-space.hh>
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
	JOINT_POSITION = 0x0,
	JACOBIAN = 0x1,
	VELOCITY = 0x2,
	ACCELERATION = 0x4,
	COM = 0x8,
	ALL = 0Xffff
      };

      /// Collision pairs between bodies
      typedef std::pair <JointPtr_t, JointPtr_t> CollisionPair_t;
      typedef std::list <CollisionPair_t> CollisionPairs_t;
      /// \name Construction, copy and destruction
      /// \{
      virtual ~Device();

      /// \brief Clone as a CkwsDevice
      DevicePtr_t clone() const;

      /// \}

      /// Get name of device
      const std::string& name () const {return name_;}

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
      virtual void rootJoint (JointPtr_t joint);

      /// Set position of root joint in world frame
      void rootJointPosition (const Transform3f& position);

      /// Get root joint
      JointPtr_t rootJoint () const;

      /// Register joint in internal containers
      void registerJoint (const JointPtr_t& joint);

      /// Get vector of joints
      const JointVector_t& getJointVector () const;

      /// Get the joint at configuration rank r
      JointPtr_t getJointAtConfigRank (const size_type& r) const;

      /// Get the joint at velocity rank r
      JointPtr_t getJointAtVelocityRank (const size_type& r) const;

      /// Get joint by name
      /// \param name name of the joint.
      /// \throw runtime_error if device has no joint with this name
      JointPtr_t getJointByName (const std::string& name) const;

      /// Get joint by body name
      /// \throw runtime_error if device has no body with this name
      JointPtr_t getJointByBodyName (const std::string& name) const;

      /// Size of configuration vectors
      /// Sum of joint dimensions and of extra configuration space dimension
      size_type configSize () const;

      /// Size of velocity vectors
      /// Sum of joint number of degrees of freedom and of extra configuration
      /// space dimension
      size_type numberDof () const;

      /// \}

      /// \name Extra configuration space
      /// \{

      /// Get degrees of freedom to store internal values in configurations
      ///
      /// In some applications, it is useful to store extra variables with
      /// the configuration vector. For instance, when planning motions in
      /// state space using roadmap based methods, the velocity of the robot
      /// is stored in the nodes of the roadmap.
      ExtraConfigSpace& extraConfigSpace () {
	return extraConfigSpace_;
      }

      /// Get degrees of freedom to store internal values in configurations
      ///
      /// In some applications, it is useful to store extra variables with
      /// the configuration vector. For instance, when planning motions in
      /// state space using roadmap based methods, the velocity of the robot
      /// is stored in the nodes of the roadmap.
      const ExtraConfigSpace& extraConfigSpace () const {
	return extraConfigSpace_;
      }

      /// Set dimension of extra configuration space
      virtual void setDimensionExtraConfigSpace (const size_type& dimension)
      {
	extraConfigSpace_.setDimension (dimension);
	resizeState (0x0);
      }

      ///

      /// \}

      /// \name Current state
      /// \{

      /// Get current configuration
      const Configuration_t& currentConfiguration () const
      {
	return currentConfiguration_;
      }
      /// Set current configuration
      /// \return True if the current configuration was modified and false if
      ///         the current configuration did not change.
      virtual bool currentConfiguration (ConfigurationIn_t configuration)
      {
	if (configuration != currentConfiguration_) {
	  upToDate_ = false;
	  currentConfiguration_ = configuration;
          return true;
	}
        return false;
      }
      /// Get the neutral configuration
      Configuration_t neutralConfiguration () const;

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
      const value_type& mass () const
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

      /// Add a gripper to the Device
      void addGripper (const GripperPtr_t& gripper)
      {
	grippers_.push_back (gripper);
      }
      /// Return list of grippers of the Device
      Grippers_t& grippers ()
      {
	return grippers_;
      }
      /// Return list of grippers of the Device
      const Grippers_t& grippers () const
      {
	return grippers_;
      }

      /// \}

      /// \name Collision and distance computation
      /// \{

      /// Get list of obstacles
      /// \param type collision or distance.
      const ObjectVector_t& obstacles (Request_t type) const;

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

      /// Remove collision pairs between objects attached to two joints
      ///
      /// \param joint1 first joint
      /// \param joint2 second joint
      /// \param type collision or distance.
      ///
      /// remove collision between each object of joint 1 body and
      /// each object of joint2 body
      virtual void removeCollisionPairs(const JointPtr_t& joint1,
                                        const JointPtr_t& joint2,
				        Request_t type);

      /// Get list of collision or distance pairs
      /// \param type collision or distance.
      const CollisionPairs_t& collisionPairs (Request_t type) const;

      /// Iterator over inner objects of the device
      /// \param type Collision or distance
      ObjectIterator objectIterator (Request_t type);

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

      /// Print object in a stream
      virtual std::ostream& print (std::ostream& os) const;

      virtual const Configuration_t q0(){return q0_;}
      virtual void q0(Configuration_t q0){q0_ = q0;}

    protected:
      /// \brief Constructor
      Device(const std::string& name);

      ///
      /// \brief Initialization.
      ///
      void init(const DeviceWkPtr_t& weakPtr);

      ///
      /// \brief Initialization of of a clone device.
      ///
      void initCopy(const DeviceWkPtr_t& weakPtr, const Device& model);


      /// Recompute number of distance pairs
      void updateDistances ();

    private:
      /// \brief Copy Constructor
      Device(const Device& device);

    private:
      void computeJointPositions ();
      void computeJointJacobians ();
      void computeMass ();
      void computePositionCenterOfMass ();
      void computeJacobianCenterOfMass ();
      void resizeState (const JointPtr_t& joint);
      void resizeJacobians ();
      std::string name_;
      DistanceResults_t distances_;
      JointByName_t jointByName_;
      JointVector_t jointVector_;
      JointVector_t jointByConfigRank_;
      JointVector_t jointByVelocityRank_;
      JointPtr_t rootJoint_;
      size_type numberDof_;
      size_type configSize_;
      Configuration_t currentConfiguration_;
      vector_t currentVelocity_;
      vector_t currentAcceleration_;
      vector3_t com_;
      ComJacobian_t jacobianCom_;
      value_type mass_;
      bool upToDate_;
      Computation_t computationFlag_;
      // Collision pairs between bodies
      CollisionPairs_t collisionPairs_;
      CollisionPairs_t distancePairs_;
      // Obstacles
      ObjectVector_t collisionObstacles_;
      ObjectVector_t distanceObstacles_;
      // Grippers
      Grippers_t grippers_;
      Configuration_t q0_;
      // Extra configuration space
      ExtraConfigSpace extraConfigSpace_;
      DeviceWkPtr_t weakPtr_;
    }; // class Device

    std::ostream& operator<< (std::ostream& os, const hpp::model::Device& device);
  } // namespace model
} // namespace hpp

#endif // HPP_MODEL_DEVICE_HH
