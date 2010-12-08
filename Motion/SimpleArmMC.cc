#include "SimpleArmMC.h"
#include "Kinematics.h"
#include "Shared/debuget.h"
#include "Shared/WorldState.h"
#include "MotionManager.h"
#include "Shared/Config.h"
#include "Wireless/Socket.h"
#include "Shared/ERS7Info.h"
#include "Shared/ERS210Info.h"
#include "Motion/IKSolver.h"
#include "Shared/get_time.h"
#include "Events/EventBase.h"

#include <memory>
#include <cmath>

SimpleArmMC::SimpleArmMC() : MotionCommand(), dirty(true){
    SimpleArmMC(0);
}

SimpleArmMC::SimpleArmMC(int index) : MotionCommand(), dirty(true){
	ArmIndex = (index - ArmOffset)/JointsPerArm;
    setWeight(1);
	defaultMaxSpeed();
	takeSnapshot();
}
void SimpleArmMC::setIndex(int index){
    ArmIndex = (index-ArmOffset)/JointsPerArm;
}

void SimpleArmMC::freezeMotion() {
#ifdef TGT_HAS_SIMPLE_ARMS
	for(unsigned int i=0; i<JointsPerArm; i++)
		armTargets[i]=armCmds[i].value;
	dirty=false;
#endif
}

void SimpleArmMC::takeSnapshot() {
#ifdef TGT_HAS_SIMPLE_ARMS
	for(unsigned int i=0; i<JointsPerArm; i++)
		armTargets[i]=armCmds[i].value=state->outputs[ArmOffset+(JointsPerArm*ArmIndex)+i];
	dirty=true;
#endif
}

void SimpleArmMC::defaultMaxSpeed(float x/*=1*/) {
#ifdef TGT_HAS_SIMPLE_ARMS
	for(unsigned int i=0; i<JointsPerArm; i++)
		maxSpeed[i]=0;
#endif
}

void SimpleArmMC::setWeight(float w) {
#ifdef TGT_HAS_SIMPLE_ARMS
	for(unsigned int x=0; x<JointsPerArm; x++)
		armCmds[x].weight=w;
	markDirty();
#endif
}

void SimpleArmMC::setJoints(float pitch, float shoulder, float elbow, float grip){
#ifdef TGT_HAS_SIMPLE_ARMS
    const char* n = NelsonInfo::outputNames[NelsonInfo::ArmOffset+(ArmIndex*NelsonInfo::JointsPerArm)+NelsonInfo::ArmShoulderPitchOffset];
    unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		armTargets[0]=clipAngularRange(i,pitch);
    n = NelsonInfo::outputNames[NelsonInfo::ArmOffset+(ArmIndex*NelsonInfo::JointsPerArm)+NelsonInfo::ArmShoulderOffset];
    i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		armTargets[1]=clipAngularRange(i,shoulder);
    n = NelsonInfo::outputNames[NelsonInfo::ArmOffset+(ArmIndex*NelsonInfo::JointsPerArm)+NelsonInfo::ArmElbowOffset];
    i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		armTargets[2]=clipAngularRange(i,elbow);
    n = NelsonInfo::outputNames[NelsonInfo::ArmOffset+(ArmIndex*NelsonInfo::JointsPerArm)+NelsonInfo::GripperOffset];
    i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		armTargets[3]=clipAngularRange(i,grip);
    markDirty();
#endif
}

int SimpleArmMC::updateOutputs() {
	int tmp=isDirty();
	if(tmp) {
		dirty=false;
#ifdef TGT_HAS_SIMPLE_ARMS
		for(unsigned int i=0; i<JointsPerArm; i++) {
			if(maxSpeed[i]<=0) {
				armCmds[i].value=armTargets[i];
				motman->setOutput(this,i+ArmOffset+(ArmIndex*JointsPerArm),armCmds[i]);
			} else { // we may be trying to exceeded maxSpeed
				unsigned int f=0;
				while(armTargets[i]>armCmds[i].value+maxSpeed[i] && f<NumFrames) {
					armCmds[i].value+=maxSpeed[i];                                        
					motman->setOutput(this,i+ArmOffset+(ArmIndex*JointsPerArm),armCmds[i],f);
					f++;
				}
				while(armTargets[i]<armCmds[i].value-maxSpeed[i] && f<NumFrames) {
					armCmds[i].value-=maxSpeed[i];
                    motman->setOutput(this,i+ArmOffset+(ArmIndex*JointsPerArm),armCmds[i],f);
					f++;
				}
				if(f<NumFrames) { //we reached target value, fill in rest of frames
					armCmds[i].value=armTargets[i];
					for(;f<NumFrames;f++)
						motman->setOutput(this,i+ArmOffset+(ArmIndex*JointsPerArm),armCmds[i],f);
				} else // we didn't reach target value, still dirty
					dirty=true;
			}
		}
#endif
	}
	return tmp;
}

int SimpleArmMC::isAlive() {
#ifndef TGT_HAS_SIMPLE_ARMS
	return false;
#else
	return true;
#endif
}

void SimpleArmMC::markDirty() {
	dirty=true;
#ifdef TGT_HAS_SIMPLE_ARMS
	for(unsigned int i=0; i<JointsPerArm; i++)
		armCmds[i].value=motman->getOutputCmd(i+ArmOffset+(ArmIndex*JointsPerArm)).value;
#endif
}

bool SimpleArmMC::ensureValidJoint(unsigned int& i) {
#ifndef TGT_HAS_SIMPLE_ARMS
	serr->printf("ERROR: SimpleArmMC received a joint index of %d on an armless target.\n",i);
#else
	if(i<JointsPerArm)
		return true;
	serr->printf("ERROR: SimpleArmMC received a joint index of %d \n",i);
	serr->printf("ERROR: This does not appear to be an arm joint.\n");
#endif
	return false;
}

