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

#ifndef HPP_MODEL_SPECIFIC_HUMANOID_ROBOT_HH
#define HPP_MODEL_SPECIFIC_HUMANOID_ROBOT_HH

#include <KineoWorks2/kwsInterface.h>
#include <KineoUtility/kitInterface.h>
#include <KineoModel/kppDeviceComponent.h>

#include <kcd2/kcdInterface.h>
#include <kwsKcd2/kwsKCDBody.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

#include "hpp/model/humanoid-robot.hh"

namespace hpp {
  namespace model {
    template <class HDR = impl::HumanoidDynamicRobot>
    class SpecificHumanoidRobot;

#define SpecificHumanoidRobotShPtr			\
    boost::shared_ptr< SpecificHumanoidRobot<HDR> >
#define SpecificHumanoidRobotWkPtr			\
    boost::weak_ptr< SpecificHumanoidRobot<HDR> >


    /// \brief Specific implementation of humanoid robot with geometric and dynamic model .

    /// This template class enables a developer to define a composite
    /// humanoid robot class based on an optimized implementation of
    /// impl::HumanoidDynamicRobot. To do so,

    /// \li derive impl::HumanoidDynamicRobot into CoptHumanoidDynamicRobot
    /// and overload the methods you want to optimize,

    /// \li define your own SpecificHumanoidRobot class by
    /// instanciating the template with your implementation:

    /// \code typedef SpecificHumanoidRobot<CoptHumanoidDynamicRobot> CyourOptHppHumanoidRobot; \endcode

    /// \image html classDiagramHumanoid.png "Inheritance diagram of a composite humanoid robot class based on an optimized implementation of the humanoid robot dynamic model."

    template <class HDR> class SpecificHumanoidRobot :
      public HDR, public HumanoidRobot
    {
    public:
      ///
      /// \name Construction, copy and destruction
      /// @{

      /// \brief Clone as a CkwsDevice
      CkwsDeviceShPtr clone() const;

      /// \brief Clone as a CkppComponent
      CkppComponentShPtr cloneComponent() const;

      /// \brief Whether component is clonable.
      /// \return true
      bool isComponentClonable() const;

      /// \brief Disambiguate parent method
      /// Call HumanoidRobot::initialize()
      virtual bool initialize();

      ///
      /// @}
      ///

      /// \brief Creation of a new humanoid robot
      /// \return a shared pointer to the new robot
      /// \param name Name of the device (is passed to CkkpDeviceComponent)
      static SpecificHumanoidRobotShPtr create(std::string name);

    protected:
      /// \brief Constructor
      /// \param objFactory factory necessary to build a CjrlDynamicHumanoidRobot.
      SpecificHumanoidRobot(CjrlRobotDynamicsObjectFactory *objFactory);

      /// \brief Initialization.
      ktStatus init(const SpecificHumanoidRobotWkPtr& weakPtr,
		    const std::string& name);

    private:

      /// \brief Store weak pointer to object.
      SpecificHumanoidRobotWkPtr weakPtr_;
    };


    template <class HDR>
    SpecificHumanoidRobot<HDR>::SpecificHumanoidRobot
    (CjrlRobotDynamicsObjectFactory *objFactory) :
      impl::DynamicRobot(), impl::HumanoidDynamicRobot(objFactory), HDR(objFactory),
      HumanoidRobot(objFactory)
    {
    }

    // =====================================================================

    template <class HDR> SpecificHumanoidRobotShPtr
    SpecificHumanoidRobot<HDR>::create(std::string name)
    {
      impl::ObjectFactory* objFactory = new impl::ObjectFactory();

      SpecificHumanoidRobot<HDR> *hppDevice =
	new SpecificHumanoidRobot<HDR>(objFactory);
      SpecificHumanoidRobotShPtr hppDeviceShPtr(hppDevice);

      if (hppDevice->init(hppDeviceShPtr, name) != KD_OK) {
	hppDeviceShPtr.reset();
      }
      return hppDeviceShPtr;
    }

    // ======================================================================

    template <class HDR> CkwsDeviceShPtr
    SpecificHumanoidRobot<HDR>::clone() const
    {
      return CkwsDeviceShPtr();
    }

    // ======================================================================

    template <class HDR> CkppComponentShPtr
    SpecificHumanoidRobot<HDR>::cloneComponent() const
    {
      return CkppComponentShPtr();
    }

    // ======================================================================

    template <class HDR> bool
    SpecificHumanoidRobot<HDR>::isComponentClonable() const
    {
      return false;
    }

    template <class HDR>
    bool SpecificHumanoidRobot<HDR>::initialize()
    {
      return HumanoidRobot::initialize();
    }

    // ======================================================================

    template <class HDR> ktStatus
    SpecificHumanoidRobot<HDR>::init
    (const SpecificHumanoidRobotWkPtr& inDevWkPtr, const std::string &name)
    {
      ktStatus success = HumanoidRobot::init(inDevWkPtr, name);

      if(KD_OK == success) {
	weakPtr_ = inDevWkPtr;
      }
      return success;
    }
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_SPECIFIC_HUMANOID_ROBOT_HH
