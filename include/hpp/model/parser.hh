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

#ifndef HPP_CORE_PARSER_HH
#define HPP_CORE_PARSER_HH

#include <KineoModel/kppComponent.h>
#include <kprParserXML/kprXMLTag.h>
#include <kprParserXML/kprXMLBuildingContext.h>
#include <kprParserXML/kprXMLWriter.h>

KIT_PREDEF_CLASS(CkppDeviceComponent);

namespace hpp {
  namespace model {
    class Parser {
    public:
      /// Add tags to Kineo parser
      /// \param addon: whether the parser is embedded in and addon application.
      /// \note Initialization is slightly different depending on whether the
      /// parser is constructed by an addon application of by a plug-in to
      /// Kitelab.
      Parser(bool addon = true);
      ~Parser();

      ///
      /// \name Call back for kxml read write
      /// @{

      /// \brief Write local class representing a humanoid robot into kxml file.
      ktStatus writeHumanoidRobot(const CkppComponentConstShPtr& inComponent,
				  CkprXMLWriterShPtr& inOutWriter,
				  CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a humanoid robot from kxml file.
      ktStatus buildHumanoidRobot(const CkprXMLTagConstShPtr& inTag,
				  const CkppComponentShPtr&
				  inOutParentComponent,
				  std::vector< CkppComponentShPtr >&
				  inPrebuiltChildComponentVector,
				  CkprXMLBuildingContextShPtr& inOutContext,
				  CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a freeflyer joint into kxml file.
      ktStatus
      writeFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
			     CkprXMLWriterShPtr& inOutWriter,
			     CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a freeflyer joint from kxml file.
      ktStatus
      buildFreeflyerJoint(const CkprXMLTagConstShPtr& inTag,
			     const CkppComponentShPtr&
			     inOutParentComponent,
			     std::vector< CkppComponentShPtr >&
			     inPrebuiltChildComponentVector,
			     CkprXMLBuildingContextShPtr& inOutContext,
			     CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a rotation joint into kxml file.
      ktStatus
      writeRotationJoint(const CkppComponentConstShPtr& inComponent,
			    CkprXMLWriterShPtr& inOutWriter,
			    CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a rotation joint from kxml file.
      ktStatus
      buildRotationJoint(const CkprXMLTagConstShPtr& inTag,
			    const CkppComponentShPtr&
			    inOutParentComponent,
			    std::vector< CkppComponentShPtr >&
			    inPrebuiltChildComponentVector,
			    CkprXMLBuildingContextShPtr& inOutContext,
			    CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a translation joint into kxml file.
      ktStatus
      writeTranslationJoint(const CkppComponentConstShPtr& inComponent,
			       CkprXMLWriterShPtr& inOutWriter,
			       CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a translation joint from kxml file.
      ktStatus
      buildTranslationJoint(const CkprXMLTagConstShPtr& inTag,
			       const CkppComponentShPtr&
			       inOutParentComponent,
			       std::vector< CkppComponentShPtr >&
			       inPrebuiltChildComponentVector,
			       CkprXMLBuildingContextShPtr& inOutContext,
			       CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a anchor joint into kxml file.
      ktStatus
      writeAnchorJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a anchor joint from kxml file.
      ktStatus
      buildAnchorJoint(const CkprXMLTagConstShPtr& inTag,
		       const CkppComponentShPtr&
		       inOutParentComponent,
		       std::vector< CkppComponentShPtr >&
		       inPrebuiltChildComponentVector,
		       CkprXMLBuildingContextShPtr& inOutContext,
		       CkppComponentShPtr& outComponent);
      /// @}
    }; // Parser
  } // namespace model
} // namespace hpp

#endif // HPP_CORE_PARSER_HH
