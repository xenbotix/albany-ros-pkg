/* Team Undergrad - Head to Head Competition Code
 * 
 * Description: Dog walks forward. Tracks to goal objects and sits down. 
 *  If a wall collision occurs, dog will:
 *   50% of time, turn 90 degrees right and continue walking
 *   30% of time, turn left until the IR sensor clears, then follow wall
 *   20% of time, explode, seriously....
 * 
 * Revisions
 * 	4/17 - 12:00PM - Created by Rob and Linsay
 *  4/18 - 11:48AM - start node is now wait state, switched to using
 *                   near IR, defined critical distance, renamed headtap to wait,
 *                   changed transition out of walkto to a timer, various other
 * 				     corrections to make it compile - MEF
 *  4/23 -  5:45PM - In class testing, fixed several errors
 *  4/24 -  2:39PM - Added repos node, repos and waiting are now SpecialWalkNodes
 *                   in which they reposition the head correctly - MEF
 */


#ifndef INCLUDED_Walk1_h_
#define INCLUDED_Walk1_h_
 
#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Motion/WalkMC.h"
#include "Motion/MMAccessor.h"
#include "Behaviors/StateNode.h"
#include "Behaviors/Nodes/WalkNode.h"
#include "Behaviors/Nodes/WalktoTargetNode.h"
#include "Behaviors/Nodes/SoundNode.h"
#include "Behaviors/Nodes/PostureNode.h"
#include "Behaviors/Transitions/SmoothCompareTrans.h"
#include "Behaviors/Transitions/TimeOutTrans.h"
#include "Behaviors/Transitions/EventTrans.h"
#include "Behaviors/Transitions/VisualTargetTrans.h"
#include "Behaviors/Transitions/CompletionTrans.h"
#include "Shared/WorldState.h"
#include "Shared/ProjectInterface.h"

#include "FollowNode.h"
#include "SpecialWalk.h"

#define CLOSE_DIST			250
#define CLEAR_DIST			400
#define FOLLOW_TIME			5500

#define LEFT 				RAD(90)
#define RIGHT				RAD(-90)
#define SLOW_LEFT 			RAD(40)
#define SLOW_RIGHT			RAD(-40)


// Change these when the time comes.
#define ENEMY_SID			ProjectInterface::visDotPinkSID   //ProjectInterface::visDotGreenSID
#define UNCLAIMED_SID		ProjectInterface::visDotOrangeSID //ProjectInterface::visDotPinkSID

class Walk1 : public StateNode {
public:
        
   Walk1() : StateNode("Walk1"), start(NULL), walk_1(NULL), right_turn(NULL), walk_2(NULL), waiting(NULL), repos(NULL),
		 walktoE(NULL), walktoU(NULL), bark(NULL), sit(NULL), squareto(NULL), follow(NULL),
	     walkid(MotionManager::invalid_MC_ID) {} //Constructor

   virtual void setup() {

     walkid = motman->addPersistentMotion(SharedObject<WalkMC>()); 

     // start node is now wait state, pounce puts the head in the right position!
     //start=addNode(waiting=new PostureNode("pounce", "pounce.pos"));
	 //start=addNode(waiting=new WalkNode(getName()+"::headtap",0,0,0));
	 start=addNode(waiting=new SpecialWalkNode(walkid));
	 addNode(repos=new SpecialWalkNode(walkid));
	 // primary node is the walking forward - goes to turn if a wall hit
	 addNode(walk_1=new WalkNode(getName()+"::walk_1",150,0,0)); 
	 // make a 90 degree right turn
     addNode(right_turn=new WalkNode(getName()+"::right_turn",0,0,SLOW_RIGHT));
	 // secondary node is walking forward - goes to squareTo/follow if a wall hit
     addNode(walk_2=new WalkNode(getName()+"::walk_2",150,0,0));
     addNode(walktoE = new WalkToTargetNode(ENEMY_SID));
	 addNode(walktoU = new WalkToTargetNode(UNCLAIMED_SID));
	 addNode(sit=new PostureNode("sit", "dispos.pos"));
     addNode(bark=new SoundNode("bark","barkhigh.wav"));
	 addNode(squareto=new WalkNode(getName() + "::squareto",0,0,SLOW_LEFT));
	 addNode(follow=new FollowNode(walkid)); 

     // transition to right turn node if IR is less than 375
     walk_1->addTransition(new SmoothCompareTrans<float>(right_turn,&state->sensors[NearIRDistOffset],CompareTrans<float>::LT,CLOSE_DIST,EventBase        
	     (EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7));          
     // if headtap go to wait state
	 walk_1->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID));  
     // switch to walk_2 every 600 ms
	 walk_1->addTransition(new TimeOutTrans(walk_2,500)); 
	 // when object is seen go to walkto
     walk_1->addTransition(new VisualTargetTrans(walktoE,ENEMY_SID));
     walk_1->addTransition(new VisualTargetTrans(walktoU,UNCLAIMED_SID));
     	 
	 // turn right for 500 ms and then go back to walk_1
	 right_turn->addTransition(new TimeOutTrans(walk_1,300));		// was 500
	 // if headtap go to wait
     right_turn->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID));
     
	 // kick back to walk_1 after 300 ms
	 walk_2->addTransition(new TimeOutTrans(walk_1,300));     
	 // if headtap go to wait
     walk_2->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID)); 
     // if object is seen go to walkto
	 walk_2->addTransition(new VisualTargetTrans(walktoE,ENEMY_SID)); 
	 walk_2->addTransition(new VisualTargetTrans(walktoU,UNCLAIMED_SID)); 
	 // if we get too close in walk2 we will follow wall (after squareing to it
	 walk_2->addTransition(new SmoothCompareTrans<float>(squareto,&state->sensors[FarIRDistOffset],CompareTrans<float>::LT,CLOSE_DIST,EventBase        
	     (EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7));    
     
	 // leave wait state only after head tap
	 waiting->addTransition(new EventTrans(walk_1,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID));  
	 
	 // when object is lost transition to sitting and barking
	 walktoU->addTransition(walktoU->newDefaultLostTrans(sit));
	 walktoE->addTransition(walktoE->newDefaultLostTrans(sit));
	 
	 // bark after sitting    
	 sit->addTransition(new CompletionTrans(bark));  
	 
	 // if we had a false detection, will go back to walk_1
     //bark->addTransition(new TimeOutTrans(walk_1, 7000)); 
	 bark->addTransition(new TimeOutTrans(repos, 7000));
	 // if headtap go to wait
     bark->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID));
	 
	 // if headtap go to wait
	 squareto->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadFrButOffset, EventBase::activateETID));
	 // turn left until IR is clear, then follow wall
	 squareto->addTransition(new SmoothCompareTrans<float>(follow,&state->sensors[NearIRDistOffset],CompareTrans<float>::GT,450,EventBase        
     	     (EventBase::sensorEGID,SensorSrcID::UpdatedSID,EventBase::statusETID),.7));
	 // timeout for failure!
	 squareto->addTransition(new TimeOutTrans(walk_1,5000));
	
	 // we go back to random walking after a short time
	 //follow->addTransition(new TimeOutTrans(walk_1, 3000)); 
	 follow->addTransition(new TimeOutTrans(repos,FOLLOW_TIME));
	 // if we see an object, walk to it
	 follow->addTransition(new VisualTargetTrans(walktoE,ENEMY_SID)); 
	 follow->addTransition(new VisualTargetTrans(walktoU,UNCLAIMED_SID)); 
	 // if headtap go to wait
	 follow->addTransition(new EventTrans(waiting,EventBase::buttonEGID, ERS7Info::HeadButOffset, EventBase::activateETID)); 
     
	 // after we reposition head, walk again
	 repos->addTransition(new TimeOutTrans(walk_1, 500));
	 
	 //Set up MC's
     walk_1->setMC(walkid);
     right_turn->setMC(walkid);
	 walk_2->setMC(walkid);
	 squareto->setMC(walkid);
     //waiting->setMC(walkid);
     StateNode::setup();
}

virtual void DoStart() {
    StateNode::DoStart();
    start->DoStart();
    std::cout << getName() << " is starting up." << std::endl;
}

virtual void DoStop() {
    std::cout << getName() << " is shutting down." << std::endl;
    erouter->removeListener(this);
    StateNode::DoStop();
}

virtual void teardown(){
   motman->removeMotion(walkid);
   StateNode::teardown();
}

virtual void processEvent(const EventBase &event) {
   // std::cout << getName() << " got event: " << event.getDescription()
    //          << std::endl;
}

protected:
    StateNode * start; //!< the node to begin within on DoStart() (turn)
    WalkNode * walk_1;
    //SpecialWalkNode * walk_1;
	WalkNode * right_turn;
    WalkNode * walk_2;
    //PostureNode * waiting;
    //WalkNode * waiting;
	SpecialWalkNode * waiting;
	SpecialWalkNode * repos;	// reposition the head after target, follow
	WalkToTargetNode * walktoE;
	WalkToTargetNode * walktoU;
	SoundNode * bark;
    PostureNode * sit;
	WalkNode * squareto;
	FollowNode * follow;    
	MotionManager::MC_ID walkid; //!< we want to share a walk between turning and walking nodes
  
private:
    Walk1(const Walk1&);
    Walk1 operator=(const Walk1&);      
};

#endif
