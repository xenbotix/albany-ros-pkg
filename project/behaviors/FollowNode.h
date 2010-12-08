/* Team Undergrad - FollowNode
 * 
 * Description: Follows a right wall, using a state machine
 *
 * Revisions:
 *  4/23 -  8:30PM - moved head_id initilization to DoStart(),
 * 			         rather than in constructor.
 */


#ifndef INCLUDED_FollowNode_h_
#define INCLUDED_FollowNode_h_
 
#include "Behaviors/StateMachine.h"

// this follows right walls walls
class FollowNode : public StateNode {
public:
        
   FollowNode() : StateNode("FollowNode"), start(NULL), leftish(NULL), rightish(NULL), 
	      walkid(MotionManager::invalid_MC_ID) {} //Constructor

   virtual void setup() {
     StateNode::setup();
	 // we get our walkId from constructor
     walkid = motman->addPersistentMotion(SharedObject<WalkMC>()); 

     // walk forward, slightly to the left
     addNode(leftish=new WalkNode(getName()+"::leftish",100,0,RAD(5)));
	 // walk forward, slightly to the right
     startnode=addNode(rightish=new WalkNode(getName()+"::rightish",100,0,RAD(-5)));

     // transition to leftish walk if IR is less than FOLLOW_DIST
     rightish->addTransition(new SmoothCompareTrans<float>(leftish,&state->sensors[NearIRDistOffset],CompareTrans<float>::LT,200,EventBase        
	     (EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7));          
     // transition to rightish walk if IR is greater than FOLLOW_DIST
	 leftish->addTransition(new SmoothCompareTrans<float>(rightish,&state->sensors[NearIRDistOffset],CompareTrans<float>::GT,200,EventBase        
	     (EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7));          
	 
	 //Set up MC's
     leftish->setMC(walkid);
     rightish->setMC(walkid);
}

virtual void teardown(){
   StateNode::teardown();
   motman->removeMotion(walk_id);
}

virtual void processEvent(const EventBase &event) {
   // std::cout << getName() << " got event: " << event.getDescription()
    //          << std::endl;
}

protected:
    StateNode * start;
    WalkNode * leftish;
    WalkNode * rightish;   
    MotionManager::MC_ID walkid; 
  
private:
    FollowNode(const FollowNode&);
    FollowNode operator=(const FollowNode&);      
};

#endif
