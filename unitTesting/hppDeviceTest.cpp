/*
 *  Copyright
 */

#include <iostream>
#include "hppDeviceTest.h" 

ChppDeviceTest::ChppDeviceTest()
{
  attChppDevice = ChppDevice::create("test");

  std::cout << "Constructor of unitTesting object of class ChppDevice." << std::endl;
}
