#include "FaceMC.h"
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

FaceMC::FaceMC() : MotionCommand(), dirty(true){
	setWeight(1);
	defaultMaxSpeed();
	takeSnapshot();
}

void FaceMC::freezeMotion() {
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headTargets[i]=headCmds[i].value;
	dirty=false;
#endif
}

void FaceMC::takeSnapshot() {
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headTargets[i]=headCmds[i].value=state->outputs[HeadOffset+i];
	dirty=true;
#endif
}

void FaceMC::defaultMaxSpeed(float x/*=1*/) {
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		maxSpeed[i]=0;
#endif
}

void FaceMC::setWeight(float w) {
#ifdef TGT_HAS_HEAD
	for(unsigned int x=0; x<NumHeadJoints; x++)
		headCmds[x].weight=w;
	markDirty();
#endif
}

void FaceMC::setNeck(float tilt, float pan, float tilt2){
#ifdef TGT_HAS_HEAD
	const char* n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::TiltOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tilt);
	n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::PanOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,pan);
	n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::NodOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tilt2);
    markDirty();
#endif
}

void FaceMC::setMouth(float tlip, float blip){
#ifdef TGT_HAS_FACE
	const char* n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::TopLipOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tlip);
    n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::BottomLipOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,blip);
    markDirty();
#endif
}

void FaceMC::setEyes(float tilt, float lpan, float rpan, float lids){
#ifdef TGT_HAS_FACE
	const char* n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::EyeTiltOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,tilt);
    n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::EyePanLeftOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,lpan);    
    n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::EyePanRightOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,rpan);
    n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::EyeLidOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,lids);
    markDirty();
#endif
}

void FaceMC::setEyebrows(float left, float right){
#ifdef TGT_HAS_FACE
	const char* n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::LeftEyebrowOffset];
	unsigned int i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,left);
    n = NelsonInfo::outputNames[NelsonInfo::HeadOffset+NelsonInfo::RightEyebrowOffset];
	i = capabilities.findOutputOffset(n);
	if(i!=-1U)
		headTargets[i-HeadOffset]=clipAngularRange(i,right);
    markDirty();
#endif
}

int FaceMC::updateOutputs() {
	int tmp=isDirty();
	if(tmp) {
		dirty=false;
#ifdef TGT_HAS_HEAD
		for(unsigned int i=0; i<NumHeadJoints; i++) {
			if(maxSpeed[i]<=0) {
				headCmds[i].value=headTargets[i];
				motman->setOutput(this,i+HeadOffset,headCmds[i]);
			} else { // we may be trying to exceeded maxSpeed
				unsigned int f=0;
				while(headTargets[i]>headCmds[i].value+maxSpeed[i] && f<NumFrames) {
					headCmds[i].value+=maxSpeed[i];                                        
					motman->setOutput(this,i+HeadOffset,headCmds[i],f);
					f++;
				}
				while(headTargets[i]<headCmds[i].value-maxSpeed[i] && f<NumFrames) {
					headCmds[i].value-=maxSpeed[i];
                                        motman->setOutput(this,i+HeadOffset,headCmds[i],f);
					f++;
				}
				if(f<NumFrames) { //we reached target value, fill in rest of frames
					headCmds[i].value=headTargets[i];
					for(;f<NumFrames;f++)
						motman->setOutput(this,i+HeadOffset,headCmds[i],f);
				} else // we didn't reach target value, still dirty
					dirty=true;
			}
		}
#endif
	}
	return tmp;
}

int FaceMC::isAlive() {
#ifndef TGT_HAS_HEAD
	return false;
#else
	return true;
#endif
}

void FaceMC::markDirty() {
	dirty=true;
#ifdef TGT_HAS_HEAD
	for(unsigned int i=0; i<NumHeadJoints; i++)
		headCmds[i].value=motman->getOutputCmd(HeadOffset+i).value; //not state->outputs[HeadOffset+i]; - see function documentation
#endif
}

bool FaceMC::ensureValidJoint(unsigned int& i) {
#ifndef TGT_HAS_HEAD
	serr->printf("ERROR: FaceMC received a joint index of %d on headless target.\n",i);
#else
	if(i<NumHeadJoints)
		return true;
	if(i>=HeadOffset && i<HeadOffset+NumHeadJoints) {
		i-=HeadOffset;
		serr->printf("WARNING: FaceMC received a joint index of %d (HeadOffset+%d).\n",i+HeadOffset,i);
		serr->printf("         Since all parameters are assumed to be relative to HeadOffset,\n");
		serr->printf("         you should just pass %d directly.\n",i);
		serr->printf("WARNING: Assuming you meant %d...\n",i);
		return true;
	}
	serr->printf("ERROR: FaceMC received a joint index of %d (HeadOffset%+d).\n",i,i-HeadOffset);
	serr->printf("ERROR: This does not appear to be a head joint.  FaceMC only controls\n");
	serr->printf("       head joints, and assumes its arguments are relative to HeadOffset\n");
#endif
	return false;
}

