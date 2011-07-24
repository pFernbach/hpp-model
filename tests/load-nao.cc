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

#include <sstream>

#define BOOST_TEST_MODULE LOAD_NAO
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
using boost::test_tools::output_test_stream;

#include <kprParserXML/kprParserManager.h>
#include <KineoModel/kppComponent.h>
#include <KineoModel/kppModelTree.h>
#include <KineoModel/kppDeviceNode.h>
#include "KineoModel/kppLicense.h"

#include <hpp/util/debug.hh>
#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/parser.hh"
#include "hpp/model/exception.hh"

using hpp::model::Exception;

namespace hpp {
  namespace model {
    static void validateLicense()
    {
      if(!CkppLicense::initialize()) {
	throw Exception("failed to validate Kineo license.");
      }
    }
  } // namespace model
} // namespace hpp

BOOST_AUTO_TEST_CASE(display)
{
  hpp::model::validateLicense();
  CkppComponentShPtr modelTreeComponent;
  hpp::model::Parser extra;
  CkprParserManagerShPtr parser = CkprParserManager::defaultManager();
  std::string filename("/home/florent/devel/nao/model/nao-hpp.kxml");
  if (parser->loadComponentFromFile(filename,
				    modelTreeComponent) != KD_OK) {
    CkprParser::Error error = parser->lastError();
    std::string message = "failed to read " + filename + ".\n"
      + std::string(error.errorMessage());
    hppDout(error, message);
    throw Exception(message);
  }
  CkppModelTreeShPtr modelTree =
    KIT_DYNAMIC_PTR_CAST(CkppModelTree,modelTreeComponent);
  if (!modelTree)
    throw Exception("Main node is not of type CkppModelTree.");
  if(!modelTree->deviceNode())
    throw Exception("No device node in the model tree.");
  unsigned int nbDevices = modelTree->deviceNode()->countChildComponents();
  if (nbDevices != 1) {
    std::ostringstream oss;
    oss << "Expecting one device in device node, got "
	<< nbDevices << ".";
    throw Exception(oss.str());
  }
  hpp::model::HumanoidRobotShPtr humanoidRobot =
    KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,
			 modelTree->deviceNode()->childComponent(0));

  if (!humanoidRobot)
    throw Exception("Device is not of type HumanoidRobot.");
  
  humanoidRobot->initialize();
  output_test_stream output;
  std::cout << *humanoidRobot;
}
