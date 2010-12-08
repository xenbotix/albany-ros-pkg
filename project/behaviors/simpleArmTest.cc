//-*-c++-*-
#ifdef TGT_NELSON

/* A Test of the SimpleArmMC
 * Copyright 2010 Michael E. Ferguson - SUNY Albany Robotics
 */

#ifndef INCLUDED_SIMPLE_ARM_TEST_h_
#define INCLUDED_SIMPLE_ARM_TEST_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/PIDMC.h"
#include "Motion/SimpleArmMC.h"
#include "IPC/SharedObject.h"
#include "Motion/MMAccessor.h"
#include "IPC/SharedObject.h"
#include "Motion/MotionPtr.h"

/* We create an ArmMC for the right arm, and move the left arm for voodoo control.  */
class simpleArmTest : public BehaviorBase {

public:
    //! default constructor, use type name as instance name
    simpleArmTest():BehaviorBase("SimpleArmMC Test Program"), pid(SharedObject<PIDMC>(1)),
    arm_id(motman->addPersistentMotion(SharedObject<SimpleArmMC>(NelsonInfo::RightArmOffset))) {}

    // Start up listeners for timers, sensors, etc
    virtual void DoStart() {
        BehaviorBase::DoStart();
        std::cout << "Simple Arm Test: starting..." << std::endl;
        // Turn off torque to our left arm!
        for(unsigned int i=0; i< NelsonInfo::JointsPerArm; i++){
            pid->setJointPowerLevel(NelsonInfo::LeftArmOffset+i,0);
        }
        addMotion(pid,PERSISTENT,MotionManager::kHighPriority);
        // Listen for a 30Hz update rate
        erouter->addListener(this,EventBase::sensorEGID);
    }

    virtual void processEvent(const EventBase &event){
        volatile float pitch = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmShoulderPitchOffset];
        volatile float shoulder = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmShoulderOffset];
        volatile float elbow = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmElbowOffset];
        volatile float gripper = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::GripperOffset];
        MMAccessor<SimpleArmMC> arm(arm_id);
        arm.mc()->setJoints(pitch, shoulder, elbow, gripper);
    }

    virtual void DoStop(){
        motman->removeMotion(arm_id);
        removeMotion(pid);
        // this "one-shot" version of doing things will restore the PIDs on our way out
        // note the explicit call to motman with a naked SharedObject instead of using
        // BehaviorBase::addMotion or MotionPtr Ñ otherwise this would be immediately
        // stopped again as the behavior exits and not have time to do anything
        motman->addPrunableMotion(SharedObject<PIDMC>(1));
        std::cout << "Simple Arm Test: terminated." << std::endl;
        BehaviorBase::DoStop();
    }

    static std::string getClassDescription() { return "SimpleArmMC Test Program"; }
    virtual std::string getDescription() const { return getClassDescription(); }

protected:
    MotionPtr<PIDMC> pid;           // need to disable left arm, derived from RecordRange.cc/RelaxBehavior.cc
    MotionManager::MC_ID arm_id; 

private:
    simpleArmTest(const simpleArmTest&);  // don't call (copy constructor)
    simpleArmTest& operator=(const simpleArmTest&); // don't call (assignment operator)
};

REGISTER_BEHAVIOR(simpleArmTest);

#endif // INCLUDED_SIMPLE_ARM_MC_h_
#endif // TGT_NELSON
