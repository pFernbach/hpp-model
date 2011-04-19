#ifndef HPPIMPLROBOTDYNAMICS_H 
#define HPPIMPLROBOTDYNAMICS_H

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/dynamics/dynamicrobot.hh>
#include <jrl/dynamics/humanoiddynamicrobot.hh>
#include <jrl/dynamics/hand.hh>
#include <jrl/dynamics/foot.hh>
#include <jrl/dynamics/joint.hh>
#include <jrl/dynamics/dynamicbody.hh>

typedef jrlDelegate::dynamicRobot CimplDynamicRobot;
typedef jrlDelegate::humanoidDynamicRobot CimplHumanoidDynamicRobot;
typedef dynamicsJRLJapan::JointFreeflyer CimplJointFreeFlyer;
typedef dynamicsJRLJapan::JointAnchor CimplJointAnchor;
typedef dynamicsJRLJapan::JointRotation CimplJointRotation;
typedef dynamicsJRLJapan::JointTranslation CimplJointTranslation;
typedef dynamicsJRLJapan::Joint CimplJoint;
typedef dynamicsJRLJapan::DynamicBody CimplBody;
typedef dynamicsJRLJapan::Hand CimplHand;
typedef dynamicsJRLJapan::ObjectFactory CimplObjectFactory;

#endif
