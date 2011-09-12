/// \page  "Porting code from version 1.x to version 2.y"
///
/// \section hpp_model_porting_intro Introduction
///
/// The main modifications in version 2.x are the following:
/// \li coding style has been homogeneized with new hpp standard,
/// \li concrete joint classes now derive from Kitelab joint classes
/// and store a pointer to an implementation of dynamic class. See detailed
/// description of class hpp::model::Joint for more information.
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
/// \li ChppDevice has been renamed into hpp::model::Device,
/// \li ChppHumanoidRobot has been renamed into hpp::model::HumanoidRobot,
/// \li ChppJoint has been renamed into hpp::model::Joint,
/// \li ChppBody has been renamed into hpp::model::Body.
/// \li ChppSpecificHumanoidRobot template class has been renamed into
/// hpp::model::SpecificHumanoidRobot
///
/// \subsection hpp_model_porting_joint_constructors Joint constructors
///
/// Joints are not anymore constructed via the device class, but have standard
/// create static methods as any Kitelab component class.
/// \note It is not recommended to use create(const std::string& name) of joint
/// implementation classes:
/// \li hpp::model::AnchorJoint::create(const std::string& name),
/// \li hpp::model::FreeflyerJoint::create(const std::string& name),
/// \li hpp::model::RotationJoint::create(const std::string& name),
/// \li hpp::model::TranslationJoint::create(const std::string& name),
///
/// since creation of dynamic part of joint is delayed until call of
/// hpp::model::HumanoidRobot::initialize().
///
/// Instead the following modification should be done in client code:
/// \li ChppDevice::createAnchor(const std::string& name,
/// const CkitMat4& initialPosition) becomes
/// hpp::model::AnchorJoint::create(const std::string& name,
/// const CkitMat4& initialPosition)
/// \li ChppDevice::createFreeFlyer(const std::string& name,
/// const CkitMat4& initialPosition) becomes
/// hpp::model::FreeflyerJoint::create(const std::string& name,
/// const CkitMat4& initialPosition)
/// \li ChppDevice::createRotation(const std::string& name,
/// const CkitMat4& initialPosition) becomes
/// hpp::model::RotationJoint::create(const std::string& name,
/// const CkitMat4& initialPosition)
/// \li ChppDevice::createTranslation(const std::string& name,
/// const CkitMat4& initialPosition) becomes
/// hpp::model::TranslationJoint::create(const std::string& name,
/// const CkitMat4& initialPosition)
///
/// \note The above "create" methods return shared pointers instead of
/// pointers. Most methods manipulating joints take as input or
/// return shared pointer to hpp::model::Joint as well. This might
/// imply some minor modifications in client code.
///
/// \subsection hpp_model_porting_body Modification in class Body
///
/// class hpp::model::Body does not derive anymore from impl::DynamicBody.
/// When creating a joint, it is now necessary to create
/// \li a hpp::model::Body object and
/// \li a impl::DynamicBody object (implementation of CjrlBody).
///
/// Method hpp::model::Joint::insertBody() does that for the user.
///
/// Methods hpp::model::Joint::setAttachedBody() and
/// hpp::model::Joint::attachedBody() have been suppressed. They can be replaced
/// by parent class CkwsJoint implementation through dynamic casting.
///
/// if <c>joint</c> is of type <c>hpp::model::JointShPtr</c>, the attached body
/// can be retrieved by:
/// \code hpp::model::BodyShPtr body = KIT_DYNAMIC_PTR_CAST(hpp::model::Joint, joint->attachedBody());\endcode
///
/// Methods hpp::model::Body::hppJoint() have been suppressed. the joint owning
/// a body can be accessed by parent class CkwsKCDBody and dynamic casting.
/// \code hpp::model::JointShPtr joint = KIT_DYNAMIC_PTR_CAST(hpp::model::Joint, body->joint());\endcode
///
