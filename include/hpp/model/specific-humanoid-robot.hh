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
    class ChppSpecificHumanoidRobot;

#define ChppSpecificHumanoidRobotShPtr			\
    boost::shared_ptr< ChppSpecificHumanoidRobot<HDR> >
#define ChppSpecificHumanoidRobotWkPtr			\
    boost::weak_ptr< ChppSpecificHumanoidRobot<HDR> >


    /// \brief Specific implementation of humanoid robot with geometric and dynamic model .

    /// This template class enables a developer to define a composite
    /// humanoid robot class based on an optimized implementation of
    /// impl::HumanoidDynamicRobot. To do so,

    /// \li derive impl::HumanoidDynamicRobot into CoptHumanoidDynamicRobot
    /// and overload the methods you want to optimize,

    /// \li define your own ChppSpecificHumanoidRobot class by
    /// instanciating the template with your implementation:

    /// \code typedef ChppSpecificHumanoidRobot<CoptHumanoidDynamicRobot> CyourOptHppHumanoidRobot; \endcode

    /// \image html classDiagramHumanoid.png "Inheritance diagram of a composite humanoid robot class based on an optimized implementation of the humanoid robot dynamic model."

    template <class HDR> class ChppSpecificHumanoidRobot :
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

      ///
      /// @}
      ///

      /// \brief Creation of a new humanoid robot
      /// \return a shared pointer to the new robot
      /// \param name Name of the device (is passed to CkkpDeviceComponent)
      static ChppSpecificHumanoidRobotShPtr create(std::string name);

    protected:
      /// \brief Constructor
      /// \param objFactory factory necessary to build a CjrlDynamicHumanoidRobot.
      ChppSpecificHumanoidRobot(CjrlRobotDynamicsObjectFactory *objFactory);

      /// \brief Initialization.
      ktStatus init(const ChppSpecificHumanoidRobotWkPtr& weakPtr,
		    const std::string& name);

      /// \brief Initialization with shared pointer.
      ktStatus init(const ChppSpecificHumanoidRobotWkPtr& weakPtr,
		    const ChppSpecificHumanoidRobotShPtr& device);

    private:

      /// \brief Store weak pointer to object.
      ChppSpecificHumanoidRobotWkPtr weakPtr_;
    };


    template <class HDR>
    ChppSpecificHumanoidRobot<HDR>::ChppSpecificHumanoidRobot
    (CjrlRobotDynamicsObjectFactory *objFactory) :
      impl::DynamicRobot(), impl::HumanoidDynamicRobot(objFactory), HDR(objFactory),
      HumanoidRobot(objFactory)
    {
    }

    // =====================================================================

    template <class HDR> ChppSpecificHumanoidRobotShPtr
    ChppSpecificHumanoidRobot<HDR>::create(std::string name)
    {
      impl::ObjectFactory* objFactory = new impl::ObjectFactory();

      ChppSpecificHumanoidRobot<HDR> *hppDevice =
	new ChppSpecificHumanoidRobot<HDR>(objFactory);
      ChppSpecificHumanoidRobotShPtr hppDeviceShPtr(hppDevice);

      if (hppDevice->init(hppDeviceShPtr, name) != KD_OK) {
	hppDeviceShPtr.reset();
      }
      return hppDeviceShPtr;
    }

    // ======================================================================

    template <class HDR> CkwsDeviceShPtr
    ChppSpecificHumanoidRobot<HDR>::clone() const
    {
      return CkwsDeviceShPtr();
    }

    // ======================================================================

    template <class HDR> CkppComponentShPtr
    ChppSpecificHumanoidRobot<HDR>::cloneComponent() const
    {
      return CkppComponentShPtr();
    }

    // ======================================================================

    template <class HDR> bool
    ChppSpecificHumanoidRobot<HDR>::isComponentClonable() const
    {
      return false;
    }

    // ======================================================================

    template <class HDR> ktStatus
    ChppSpecificHumanoidRobot<HDR>::init
    (const ChppSpecificHumanoidRobotWkPtr& inDevWkPtr, const std::string &name)
    {
      ktStatus success = Device::init(inDevWkPtr, name);

      if(KD_OK == success) {
	weakPtr_ = inDevWkPtr;
      }
      return success;
    }

    // ======================================================================

    template <class HDR> ktStatus
    ChppSpecificHumanoidRobot<HDR>::init
    (const ChppSpecificHumanoidRobotWkPtr& weakPtr,
     const ChppSpecificHumanoidRobotShPtr& device)
    {
      ktStatus  success = Device::init(weakPtr, device);

      if(KD_OK == success) {
	weakPtr_ = weakPtr;
      }

      return success;
    }
  } // namespace model
} // namespace hpp
#endif // HPP_MODEL_SPECIFIC_HUMANOID_ROBOT_HH
