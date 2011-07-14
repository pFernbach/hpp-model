#ifndef HPPIMPLROBOTDYNAMICS_H 
#define HPPIMPLROBOTDYNAMICS_H

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl/dynamics/dynamicrobot.hh>
#include <jrl/dynamics/humanoiddynamicrobot.hh>
#include <jrl/dynamics/hand.hh>
#include <jrl/dynamics/foot.hh>
#include <jrl/dynamics/joint.hh>
#include <jrl/dynamics/dynamicbody.hh>

namespace hpp {
  namespace model {
    namespace impl {
    typedef jrlDelegate::dynamicRobot DynamicRobot;
    typedef jrlDelegate::humanoidDynamicRobot HumanoidDynamicRobot;
    typedef dynamicsJRLJapan::JointFreeflyer JointFreeflyer;
    typedef dynamicsJRLJapan::JointAnchor JointAnchor;
    typedef dynamicsJRLJapan::JointRotation JointRotation;
    typedef dynamicsJRLJapan::JointTranslation JointTranslation;
    typedef dynamicsJRLJapan::Joint Joint;
    typedef dynamicsJRLJapan::DynamicBody DynamicBody;
    typedef dynamicsJRLJapan::Hand Hand;
    typedef dynamicsJRLJapan::ObjectFactory ObjectFactory;
    } // namespace impl
  } // namespace model
} // namespace hpp
#endif
