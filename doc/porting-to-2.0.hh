/// \page  "Porting code from version 1.x to version 2.y"
///
/// \section hpp_model_porting_intro Introduction
///
/// The main modifications in version 2.x are the following:
/// \li coding style has been homogeneized with new hpp standard,
/// \li concrete joint classes now derive both from KineoPathPlanner classes
/// and from the dynamic implementation classes.
///
/// \section hpp_model_porting_modification Modification in client code
///
/// \li ChppDevice has been renamed into hpp::model::Device
/// \li ChppHumanoidRobot has been renamed into hpp::model::HumanoidRobot
/// \li ChppJoint has been renamed into hpp::model::Joint

