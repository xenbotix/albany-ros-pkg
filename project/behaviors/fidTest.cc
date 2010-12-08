//-*-c++-*-
#ifndef INCLUDED_FIDTEST_h_
#define INCLUDED_FIDTEST_h_

#include "Behaviors/BehaviorBase.h"
#include "Events/EventRouter.h"
#include "Events/TextMsgEvent.h"

//! DESCRIPTION
class fidTest : public BehaviorBase {

public:
	//! default constructor, use type name as instance name
	fidTest():BehaviorBase("fidTest"){}

    // Start up listeners for timers, sensors, etc
    virtual void DoStart() {
        BehaviorBase::DoStart();
	    erouter->addListener(this,EventBase::textmsgEGID);
    }

    virtual void processEvent(const EventBase &event){
	    if(event.getGeneratorID() == EventBase::textmsgEGID){
            const TextMsgEvent &txtev=dynamic_cast<const TextMsgEvent&>(event);
            std::cout << txtev.getText() << std::endl;
        }
    }

	static std::string getClassDescription() { return "Listen for FID recognition messages"; }
	virtual std::string getDescription() const { return getClassDescription(); }

	// ****************************
	// ********* MEMBERS **********
	// ****************************
protected:

private:
	// Providing declarations for these functions will avoid a compiler warning if
	// you have any class members which are pointers.  However, as it is, an error
	// will result if you inadvertantly cause a call to either (which is probably
	// a good thing, unless you really intended to copy/assign a behavior, in
	// which case simply provide implementations for the functions)
	fidTest(const fidTest&); //!< don't call (copy constructor)
	fidTest& operator=(const fidTest&); //!< don't call (assignment operator)
};

REGISTER_BEHAVIOR(fidTest);

#endif
