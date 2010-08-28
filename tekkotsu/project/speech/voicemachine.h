/* Daniel Spinner
   CSI445-Robotics
   Final Project: Behavior to demo voice recognition
*/

//-*-c++-*-
#ifndef INCLUDED_voicemachine_h_
#define INCLUDED_voicemachine_h_

// This is an empty StateNode template file.
//
// StateNodes are recursive data structures, can be used as either a leaf node
// or a machine.  This template is the suggested form for a state machine, which
// breaks a task down into one or more subnodes.
//
// You are of course welcome to combine the abilities of a leaf node (actual
// execution) and a state machine (which breaks the task down into subnodes) --
// simply add processEvent() and DoStop(), as seen in statenode.h
//
// Replace Daniel Spinner, voicemachine, and DESCRIPTION as appropriate, and go to town!


#include "Behaviors/StateNode.h"
#include "Events/EventRouter.h"
#include "Behaviors/StateMachine.h"
#include "Motion/MotionManager.h"
#include "Motion/WalkMC.h"

//! DESCRIPTION
class voicemachine : public StateNode {

	// ****************************
	// ******* CONSTRUCTORS *******
	// ****************************
public:
	//! default constructor, use type name as instance name
	voicemachine()
		: StateNode("voicemachine","voicemachine"), start(NULL), walkid(MotionManager::invalid_MC_ID)
	{}

	//! constructor, take an instance name
	voicemachine(const std::string& nm)
		: StateNode("voicemachine",nm), start(NULL), walkid(MotionManager::invalid_MC_ID)
	{}

	//! destructor, check call to teardown -- only actually necessary if you override teardown()
	~voicemachine() {
		if(issetup) 
			teardown();
	}

protected:
	//! constructor for subclasses (which would need to provide a different class name)
	voicemachine(const std::string &class_name, const std::string &node_name)
		: StateNode(class_name,node_name), start(NULL), walkid(MotionManager::invalid_MC_ID)
	{}
	
	
	// ****************************
	// ********* METHODS **********
	// ****************************
public:
	//! This function should wire together any subnodes which you may desire
	virtual void setup() {
		StateNode::setup();
		
		walkid = motman->addPersistentMotion(SharedObject<WalkMC>());

/****************************** Create the various nodes *******************************/
		WalkNode *listen_name_node; 
    	start=addNode(listen_name_node = new WalkNode("listen for name", 0, 0, 0));
    		
    	WalkNode *get_cmd_node; 
    	addNode(get_cmd_node = new WalkNode("listen for command", 0, 0, 0));
    		
    	WalkNode *dance_node; 
    	addNode(dance_node = new WalkNode("dance", 0, 0, 0));
    		
    	WalkNode *roll_node; 
    	addNode(roll_node = new WalkNode("roll over", 0, 0, 0));
    		
    	WalkNode *joke01_node; 
    	addNode(joke01_node = new WalkNode("joke 1", 0, 0, 0));
    		
    	WalkNode *joke02_node; 
    	addNode(joke02_node = new WalkNode("joke 2", 0, 0, 0));
    		
    	WalkNode *joke03_node; 
    	addNode(joke03_node = new WalkNode("joke 3", 0, 0, 0));
    		
    	WalkNode *fetch_node; 
    	addNode(fetch_node = new WalkNode("fetch", 0, 0, 0));
    	
    	WalkNode *gitmb_node; 
    	addNode(gitmb_node = new WalkNode("gitmb", 0, 0, 0));
    	
    	WalkNode *stutter_node; 
    	addNode(stutter_node = new WalkNode("stutter", 0, 0, 0));
    	
    	WalkNode *done_node; 
    	addNode(done_node = new WalkNode("done", 0, 0, 0));
    	
    	WalkNode *youre_wel_node; 
    	addNode(youre_wel_node = new WalkNode("you're welcome", 0, 0, 0));
    	
    		
    		
    		
    		
/******************************* Create the transitions *********************************/

	TextMsgTrans *name_to_cmd = new TextMsgTrans(get_cmd_node, "CHARLES");
	TimeOutTrans *cmd_to_name = new TimeOutTrans(listen_name_node, 30000);
	
	TextMsgTrans *name_to_gitmb = new TextMsgTrans(gitmb_node, "CHARLES RICK JAMES");
	TimeOutTrans *gitmb_to_cmd = new TimeOutTrans(get_cmd_node, 9000);
	
	TextMsgTrans *cmd_to_dance01 = new TextMsgTrans(dance_node, "DANCE");
	TextMsgTrans *cmd_to_dance02 = new TextMsgTrans(dance_node, "DANCE PLEASE");
	TextMsgTrans *name_to_dance01 = new TextMsgTrans(dance_node, "CHARLES DANCE");
	TextMsgTrans *name_to_dance02 = new TextMsgTrans(dance_node, "CHARLES DANCE PLEASE");
	TimeOutTrans *dance_to_cmd = new TimeOutTrans(get_cmd_node, 10000);
		
	TextMsgTrans *cmd_to_roll01 = new TextMsgTrans(roll_node, "ROLLOVER");
	TextMsgTrans *cmd_to_roll02 = new TextMsgTrans(roll_node, "ROLLOVER PLEASE");
	TextMsgTrans *name_to_roll01 = new TextMsgTrans(roll_node, "CHARLES ROLLOVER");
	TextMsgTrans *name_to_roll02 = new TextMsgTrans(roll_node, "CHARLES ROLLOVER PLEASE");
	TimeOutTrans *roll_to_cmd = new TimeOutTrans(get_cmd_node, 3000);
		
	TextMsgTrans *cmd_to_joke01 = new TextMsgTrans(joke01_node, "JOKE PLEASE");
	TextMsgTrans *cmd_to_joke02 = new TextMsgTrans(joke02_node, "JOKE PLEASE CHARLES");
	TextMsgTrans *cmd_to_joke03 = new TextMsgTrans(joke03_node, "JOKE CHARLES");
	TimeOutTrans *joke03_to_cmd = new TimeOutTrans(get_cmd_node, 9000);
	TimeOutTrans *joke01_to_cmd = new TimeOutTrans(get_cmd_node, 7500);
	TimeOutTrans *joke02_to_cmd = new TimeOutTrans(get_cmd_node, 5500);
	
	TextMsgTrans *cmd_to_done = new TextMsgTrans(done_node, "THATS ALL");
	TextMsgTrans *name_to_done = new TextMsgTrans(done_node, "THATS ALL");
	TimeOutTrans *done_to_cmd = new TimeOutTrans(get_cmd_node, 2000);
	
	TextMsgTrans *cmd_to_fetch01 = new TextMsgTrans(fetch_node, "FETCH");
	TextMsgTrans *cmd_to_fetch02 = new TextMsgTrans(fetch_node, "FETCH PLEASE");
	TextMsgTrans *name_to_fetch01 = new TextMsgTrans(fetch_node, "CHARLES FETCH");
	TextMsgTrans *name_to_fetch02 = new TextMsgTrans(fetch_node, "CHARLES FETCH PLEASE");
	TimeOutTrans *fetch_to_cmd = new TimeOutTrans(get_cmd_node, 2000);
	
	TextMsgTrans *cmd_to_stu = new TextMsgTrans(stutter_node, "WHAT");
	TimeOutTrans *stu_to_cmd = new TimeOutTrans(get_cmd_node, 2000);
	
	TextMsgTrans *cmd_to_yw01 = new TextMsgTrans(youre_wel_node, "THANK YOU");
	TextMsgTrans *cmd_to_yw02 = new TextMsgTrans(youre_wel_node, "THANK YOU CHARLES");
	TimeOutTrans *yw_to_cmd = new TimeOutTrans(get_cmd_node, 2000);
	
	
	
	
		
		
		
		
		
		
/******************************* Add transitions to nodes *******************************/
    listen_name_node->addTransition(name_to_cmd);
	listen_name_node->addTransition(name_to_dance01);
	listen_name_node->addTransition(name_to_dance02);
	listen_name_node->addTransition(name_to_gitmb);
	listen_name_node->addTransition(name_to_roll01);
	listen_name_node->addTransition(name_to_roll02);
	listen_name_node->addTransition(name_to_done);
	listen_name_node->addTransition(name_to_fetch01);
	listen_name_node->addTransition(name_to_fetch02);
		
	get_cmd_node->addTransition(cmd_to_name);
	get_cmd_node->addTransition(cmd_to_dance01);
	get_cmd_node->addTransition(cmd_to_dance02);
	get_cmd_node->addTransition(cmd_to_roll01);
	get_cmd_node->addTransition(cmd_to_roll02);
	get_cmd_node->addTransition(cmd_to_joke01);
	get_cmd_node->addTransition(cmd_to_joke02);
	get_cmd_node->addTransition(cmd_to_joke03);
	get_cmd_node->addTransition(cmd_to_done);
	get_cmd_node->addTransition(cmd_to_fetch01);
	get_cmd_node->addTransition(cmd_to_fetch02);
	get_cmd_node->addTransition(cmd_to_stu);
	get_cmd_node->addTransition(cmd_to_yw01);
	get_cmd_node->addTransition(cmd_to_yw02);
		
	dance_node->addTransition(dance_to_cmd);
		
	roll_node->addTransition(roll_to_cmd);
	
	fetch_node->addTransition(fetch_to_cmd);
	
    gitmb_node->addTransition(gitmb_to_cmd);
    
    stutter_node->addTransition(stu_to_cmd);
    
    done_node->addTransition(done_to_cmd);
    
    youre_wel_node->addTransition(yw_to_cmd);
		
	joke01_node->addTransition(joke01_to_cmd);
	joke02_node->addTransition(joke02_to_cmd);
	joke03_node->addTransition(joke03_to_cmd);
		
		
		
		
		
/******************************** Add sounds to transitions **********************************/
    	name_to_cmd->setSound("name_to_cmd.wav");
    	name_to_dance01->setSound("dance.wav");
    	name_to_dance02->setSound("dance.wav");
    	name_to_gitmb->setSound("gitmb.wav");
    	name_to_roll01->setSound("roll.wav");
        name_to_roll02->setSound("roll.wav");
        name_to_done->setSound("done.wav");
        name_to_fetch01->setSound("fetch.wav");
    	name_to_fetch02->setSound("fetch.wav");
    	
    	cmd_to_name->setSound("fart.wav");
    	cmd_to_dance01->setSound("dance.wav");
    	cmd_to_dance02->setSound("dance.wav");
    	cmd_to_roll01->setSound("cmd_to_roll.wav");
    	cmd_to_roll02->setSound("cmd_to_roll.wav");
    	cmd_to_joke01->setSound("joke01.wav");
    	cmd_to_joke02->setSound("joke02.wav");
    	cmd_to_joke03->setSound("joke03.wav");
    	cmd_to_done->setSound("done.wav");
    	cmd_to_fetch01->setSound("fetch.wav");
    	cmd_to_fetch02->setSound("fetch.wav");
    	cmd_to_stu->setSound("stutter.wav");
    	cmd_to_yw01->setSound("youre_welcome.wav");
    	cmd_to_yw02->setSound("youre_welcome.wav");
    	



        listen_name_node->setMC(walkid);
        get_cmd_node->setMC(walkid);
        dance_node->setMC(walkid);
        roll_node->setMC(walkid);
        joke01_node->setMC(walkid);
        joke02_node->setMC(walkid);
        joke03_node->setMC(walkid);
        fetch_node->setMC(walkid);
        gitmb_node->setMC(walkid);
        stutter_node->setMC(walkid);
        done_node->setMC(walkid);
        youre_wel_node->setMC(walkid);
        
        startnode = start;
    		

	}

	//! You may not need this function if the only memory allocated in
	//! setup() was subnodes and transitions
	virtual void teardown() {
		// <your teardown code here>
		StateNode::teardown(); // may delete subnodes (required)
	}

	static std::string getClassDescription() { return "DESCRIPTION"; }
	virtual std::string getDescription() const { return getClassDescription(); }


	// ****************************
	// ********* MEMBERS **********
	// ****************************
protected:
	// <class members go here>
	StateNode *start;
    MotionManager::MC_ID walkid;


	// ****************************
	// ********** OTHER ***********
	// ****************************
private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	voicemachine(const voicemachine&); //!< don't call (copy constructor)
	voicemachine& operator=(const voicemachine&); //!< don't call (assignment operator)
};

/*! @file
 * @brief Defines voicemachine, which DESCRIPTION
 * @author Daniel Spinner (Creator)
 *
 * $Author$
 * $Name$
 * $Revision$
 * $State$
 * $Date$
 */

#endif
