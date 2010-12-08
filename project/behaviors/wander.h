/*  Robotics 445/600 Spring 2010 - Wander Behavior Sample
 *  Michael Ferguson
 *  
 *  Behavior Description:
 *    Implements a right wall following behavior on the CREATE
 *    if we see a pink ball, we stop driving for 5 seconds. 
 */
 
#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/MMAccessor.h"
#include "Motion/MotionManager.h"
#include "Motion/WalkMC.h"
#include "Shared/ProjectInterface.h"

#define REG_SPEED     70          // forward speed, 70mm per second
#define STOPTURNING   4242        
#define SAFE_DIST       150

/* Conditional walk behavior */
class bWander : public BehaviorBase {
protected:
  MotionManager::MC_ID walk_id;
  int turning;
   
  /* Helper function that starts walking */
  void startWalk(double x, double angle){
    // need to call this every time, right before we edit the motion 
command.
    MMAccessor<WalkMC> walk(walk_id);
    if(done == 0){
        walk.mc()->setTargetVelocity(x,0,angle);
    }
  }
 
  /* Helper function that stops walking */
  void stopWalk(){
    MMAccessor<WalkMC> walk(walk_id);
    walk.mc()->setTargetVelocity(0,0,0);
  }

public:
  bWander() : BehaviorBase("Wander"),turning(0),
    walk_id(motman->addPersistentMotion(SharedObject<WalkMC>())){}

  /* Start up listeners for timers, sensors, etc */
  virtual void DoStart() {
    std::cout << "Wander Behavior Started." << std::endl;

    // sensor, bumper listeners
    erouter->addListener(this,EventBase::sensorEGID);
    
erouter->addListener(this,EventBase::buttonEGID,RobotInfo::BumpLeftButOffset);
    
erouter->addListener(this,EventBase::buttonEGID,RobotInfo::BumpRightButOffset);
  }
 
  virtual void DoStop() {
    motman->removeMotion(walk_id);
    BehaviorBase::DoStop();
  }  
 
  // where stuff really happens...
  virtual void processEvent(const EventBase &event){
    switch( event.getGeneratorID() ){
      // sensor values updated every 32 milliseconds - causes Cruise()
      case EventBase::sensorEGID:
        if(turning == 0){
          if(state->sensors[RobotInfo::WallSignalOffset] < SAFE_DIST)
            startWalk(REG_SPEED,.12);
          else
            startWalk(REG_SPEED,-.12);
        }
        break;
   
      // handle timer events
      case EventBase::timerEGID:
        if(event.getSourceID() == STOPTURNING){
          startWalk(REG_SPEED,0);
          turning = 0;
        }
        break;
      
      case EventBase::buttonEGID:
        // TURN!
        erouter->addTimer(this, STOPTURNING, 3000,false);
        turning = 1;
        startWalk(0,1);
        break;      
       
      // all other events
      default:
        break;
    }
  }
 
private:
    bWander(const bWander&);
    bWander operator=(const bWander&);  
};
