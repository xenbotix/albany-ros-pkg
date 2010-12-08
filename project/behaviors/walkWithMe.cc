//-*-c++-*-
#ifdef TGT_NELSON

/* Walk With Me! - A Nelson Demo
 * Copyright 2010 Michael E. Ferguson - SUNY Albany Robotics
 */

#ifndef INCLUDED_WALK_WITH_ME_h_
#define INCLUDED_WALK_WITH_ME_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/PIDMC.h"
#include "Motion/WalkMC.h"
#include "Motion/HeadPointerMC.h"
#include "IPC/SharedObject.h"
#include "Motion/MotionManager.h"
#include "Motion/MotionPtr.h"

/* Hold nelson's hand and pull him around behind you! */
class walkWithMe : public BehaviorBase {

public:
    //! default constructor, use type name as instance name
    walkWithMe():BehaviorBase("Walk With Me"), pid(SharedObject<PIDMC>(1)),
    walk_id(motman->addPersistentMotion(SharedObject<WalkMC>())),
    head_id(motman->addPersistentMotion(SharedObject<HeadPointerMC>())) {}

    // Start up listeners for timers, sensors, etc
    virtual void DoStart() {
        BehaviorBase::DoStart();
        std::cout << "Walk With Me: relaxing arm servos..." << std::endl;
        // Turn off torque to our arms!
        for(unsigned int i=0; i< 8; i++){
            pid->setJointPowerLevel(NelsonInfo::ArmOffset+i,0);
        }
        addMotion(pid,PERSISTENT,MotionManager::kHighPriority);
        // Listen for a 30Hz update rate
        erouter->addListener(this,EventBase::sensorEGID);
    }

    virtual void processEvent(const EventBase &event){
        volatile float armElev = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmShoulderPitchOffset];
        volatile float armAngle = state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmElbowOffset]
             - state->outputs[NelsonInfo::LeftArmOffset+NelsonInfo::ArmShoulderOffset] ;
        float x_speed = 0;
        float r_speed = 0;        
        if(armElev > 1.57){
            // move forward
            x_speed = (armElev-1.57) * 500;
            r_speed = armAngle * 2;
        }
        //std::cout << "WWM: " << armElev << "," << armAngle << std::endl;
        MMAccessor<WalkMC> walk(walk_id);
        walk.mc()->setTargetVelocity(x_speed,0,r_speed);
    }

    virtual void DoStop(){
        motman->removeMotion(walk_id);
        motman->removeMotion(head_id);
        removeMotion(pid);
        // this "one-shot" version of doing things will restore the PIDs on our way out
        // note the explicit call to motman with a naked SharedObject instead of using
        // BehaviorBase::addMotion or MotionPtr Ñ otherwise this would be immediately
        // stopped again as the behavior exits and not have time to do anything
        motman->addPrunableMotion(SharedObject<PIDMC>(1));
        std::cout << "Walk With Me: terminated." << std::endl;
        BehaviorBase::DoStop();
    }

    static std::string getClassDescription() { return "Walk with me!"; }
    virtual std::string getDescription() const { return getClassDescription(); }

protected:
    MotionPtr<PIDMC> pid;           // need to disable arms, derived from RecordRange.cc/RelaxBehavior.cc
    MotionManager::MC_ID walk_id;
    MotionManager::MC_ID head_id; 

private:
    walkWithMe(const walkWithMe&);  // don't call (copy constructor)
    walkWithMe& operator=(const walkWithMe&); // don't call (assignment operator)
};

REGISTER_BEHAVIOR(walkWithMe);

#endif // INCLUDED_WALK_WITH_ME_h_
#endif // TGT_IS_NELSON
