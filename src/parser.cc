//
// Copyright (c) 2010, 2011 CNRS
// Authors: Florent Lamiraux
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

#include <iostream>
#include <typeinfo>
#include <kprParserXML/kprParserManager.h>
#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/freeflyer-joint.hh"
#include "hpp/model/rotation-joint.hh"
#include "hpp/model/translation-joint.hh"
#include "hpp/model/parser.hh"

namespace hpp {
  namespace model {
    Parser::Parser()
    {
      ktStatus status = KD_ERROR;
      // Write humanoid robot
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeHumanoidRobot);
      // Read humanoid robot
      status =
	CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
	("HPP_HUMANOID_ROBOT", "DEVICE", this, &Parser::buildHumanoidRobot, NULL);
      assert(status == KD_OK);
      // Write freeflyer joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeFreeflyerJoint);
      // Read freeflyer joint
      status =
	CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
	("HPP_FREEFLYER_JOINT", "FREEFLYER_JOINT", this,
	 &Parser::buildFreeflyerJoint, NULL);
      assert(status == KD_OK);
      // Write rotation joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeRotationJoint);
      // Read rotation joint
      status =
	CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
	("HPP_ROTATION_JOINT", "ROTATION_JOINT", this,
	 &Parser::buildRotationJoint, NULL);
      assert(status == KD_OK);
      // Write translation joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeTranslationJoint);
      // Read translation joint
      status =
	CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
	("HPP_TRANSLATION_JOINT", "TRANSLATION_JOINT", this,
	 &Parser::buildTranslationJoint, NULL);
      assert(status == KD_OK);
    }

    Parser::~Parser()
    {
    }

    ktStatus Parser::writeHumanoidRobot
    (const CkppComponentConstShPtr& inComponent,
     CkprXMLWriterShPtr& inOutWriter,
     CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const HumanoidRobot, inComponent)) {
	inOutTag->name("HPP_HUMANOID_ROBOT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::buildHumanoidRobot
    (const CkprXMLTagConstShPtr& inTag,
     const CkppComponentShPtr& inOutParentComponent,
     std::vector< CkppComponentShPtr >& inPrebuiltChildComponentVector,
     CkprXMLBuildingContextShPtr& inOutContext,
     CkppComponentShPtr& outComponent)
    {
      outComponent = HumanoidRobot::create("Humanoid Robot");
      return KD_OK;
    }

    ktStatus Parser::
    writeFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
			CkprXMLWriterShPtr& inOutWriter,
			CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const FreeflyerJoint, inComponent)) {
	inOutTag->name("HPP_FREEFLYER_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildFreeflyerJoint(const CkprXMLTagConstShPtr& inTag,
			const CkppComponentShPtr& inOutParentComponent,
			std::vector< CkppComponentShPtr >&
			inPrebuiltChildComponentVector,
			CkprXMLBuildingContextShPtr& inOutContext,
			CkppComponentShPtr& outComponent)
    {
      outComponent = FreeflyerJoint::create("FREEFLYER");
      return KD_OK;
    }

    ktStatus Parser::
    writeRotationJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const RotationJoint, inComponent)) {
	inOutTag->name("HPP_ROTATION_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildRotationJoint(const CkprXMLTagConstShPtr& inTag,
		       const CkppComponentShPtr& inOutParentComponent,
		       std::vector< CkppComponentShPtr >&
		       inPrebuiltChildComponentVector,
		       CkprXMLBuildingContextShPtr& inOutContext,
		       CkppComponentShPtr& outComponent)
    {
      outComponent = RotationJoint::create("ROTATION");
      return KD_OK;
    }

    ktStatus Parser::
    writeTranslationJoint(const CkppComponentConstShPtr& inComponent,
			  CkprXMLWriterShPtr& inOutWriter,
			  CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const TranslationJoint, inComponent)) {
	inOutTag->name("HPP_TRANSLATION_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildTranslationJoint(const CkprXMLTagConstShPtr& inTag,
			  const CkppComponentShPtr& inOutParentComponent,
			  std::vector< CkppComponentShPtr >&
			  inPrebuiltChildComponentVector,
			  CkprXMLBuildingContextShPtr& inOutContext,
			  CkppComponentShPtr& outComponent)
    {
      outComponent = TranslationJoint::create("TRANSLATION");
      return KD_OK;
    }

    CkppDeviceComponentShPtr Parser::buildDummyDevice()
    {
      // Create a humanoid robot with one freeflyer joint
      HumanoidRobotShPtr robot = HumanoidRobot::create("TEST");
      FreeflyerJointShPtr joint = FreeflyerJoint::create("FF");
      joint->mass->value(1.55);
      robot->setRootJoint(joint);
      return robot;
    }

  } // namespace model
} // namespace hpp
