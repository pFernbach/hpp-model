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
/// \subsection hpp_model_porting_header_renaming Header renaming
///
/// \li header <c>hppModel/hppDevice.h</c> has been replaced by <c>hpp/model/device.hh</c>.
/// \li header <c>hppModel/hppBody.h</c> has been replaced by <c>hpp/model/body.hh</c>
/// \li header <c>hppModel/hppHumanoidRobot.h</c> has been replaced by <c>hpp/model/humanoid-robot.hh</c>
/// \li header <c>hppModel/hppJoint.h</c> has been replaced by <c>hpp/model/joint.hh</c>
/// \li header <c>hppModel/hppImplRobotDynamics.h</c> has been replaced by <c>hpp/model/robot-dynamics-impl.hh</c>
/// \li header <c>hppModel/hppSpecificHumanoidRobot.h</c> has been replaced by <c>hpp/model/specific-humanoid-robot.hh</c>
///
/// \subsection hpp_model_porting_class_renaming Class renaming
///
/// \li ChppDevice has been renamed into hpp::model::Device
/// \li ChppHumanoidRobot has been renamed into hpp::model::HumanoidRobot
/// \li ChppJoint has been renamed into hpp::model::Joint

