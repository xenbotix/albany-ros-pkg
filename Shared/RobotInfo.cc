#include "RobotInfo.h"
#include <iostream>

#if defined(TGT_ERS2xx) && defined(PLATFORM_APERIOS)
#  include <OPENR/OPENRAPI.h>
#endif

// collecting these static allocations here so we don't have to have a separate file for each one
// you can either make a .cc file dedicated to your Info.h, or just add an entry below...

#include "NelsonInfo.h"
namespace NelsonInfo {
	const char* const TargetName="Nelson";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "ERS210Info.h"
namespace ERS210Info {
	const char* const TargetName="ERS-210";
	const ERS210Capabilities capabilities;
}

#include "ERS220Info.h"
namespace ERS220Info {
	const char* const TargetName="ERS-220";
	const ERS220Capabilities capabilities;
}

#include "ERS2xxInfo.h"
namespace ERS2xxInfo {
	const char* const TargetName="ERS-2xx";
	const ERS2xxCapabilities capabilities;
}

#include "ERS7Info.h"
namespace ERS7Info {
	const char* const TargetName="ERS-7";
	const ERS7Capabilities capabilities;
}

#include "LynxArm6Info.h"
namespace LynxArm6Info {
	const char* const TargetName="LynxArm6";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,0,0,NumOutputs);
}

#include "Regis1Info.h"
namespace Regis1Info {
	const char* const TargetName="Regis1";
	const Regis1Capabilities capabilities;
}

#include "QBotPlusInfo.h"
namespace QBotPlusInfo {
	const char* const TargetName="QBotPlus";
	const QBotPlusCapabilities capabilities;
}

#include "QwerkInfo.h"
namespace QwerkInfo {
	const char* const TargetName="Qwerk";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "CreateInfo.h"
namespace CreateInfo {
	const char* const TargetName="Create";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "CalliopeInfo.h"
namespace CalliopeInfo {
	const char* const TargetName="Calliope";
	const CalliopeCapabilities capabilities;
}

#include "WiiMoteInfo.h"
namespace WiiMoteInfo {
	const char* const TargetName="WiiMote";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "BioloidInfo.h"
namespace BioloidInfo {
	const char* const TargetName="Bioloid";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "PanTiltInfo.h"
namespace PanTiltInfo {
	const char* const TargetName="PanTilt";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "HandEyeInfo.h"
namespace HandEyeInfo {
	const char* const TargetName="HandEye";
	const HandEyeCapabilities capabilities;
}

#include "HandEyeZInfo.h"
namespace HandEyeZInfo {
	const char* const TargetName="HandEyeZ";
	const HandEyeZCapabilities capabilities;
}


#include "TentacleInfo.h"
namespace TentacleInfo {
	const char* const TargetName="Tentacle";
	const Capabilities capabilities(TargetName,NumReferenceFrames,outputNames,NumButtons,buttonNames,NumSensors,sensorNames,PIDJointOffset,NumPIDJoints,LEDOffset,NumLEDs,NumOutputs);
}

#include "ChiaraInfo.h"
namespace ChiaraInfo {
	const char* const TargetName="Chiara";
	const ChiaraCapabilities capabilities;
}

// and now for RobotInfo's own stuff:
namespace RobotInfo {
	
	const char* const detectModel() {
#ifdef TGT_ERS2xx
#  ifdef PLATFORM_APERIOS
		// might be running on either 210 or 220, check
		char robotDesignStr[orobotdesignNAME_MAX + 1];
		memset(robotDesignStr, 0, sizeof(robotDesignStr));
		if (OPENR::GetRobotDesign(robotDesignStr) != oSUCCESS) {
			std::cout << "OPENR::GetRobotDesign() failed." << std::endl;
			return TargetName;
		} else {
			if(strcmp(robotDesignStr,"ERS-210")==0)
				return ERS210Info::TargetName;
			else if(strcmp(robotDesignStr,"ERS-220")==0)
				return ERS220Info::TargetName;
			else {
				std::cerr << "ERROR: Unknown name '" << robotDesignStr << "' for target ERS2xx" << std::endl;
				return TargetName;
			}
		}
#  else
#    warning TGT_ERS2xx assuming ERS-210 for simulation on local platform
		return ERS210Info::TargetName;
#  endif
		
#else
		// target is directly the robot, return the target name
		return TargetName;
#endif
	}
	
#ifndef PLATFORM_APERIOS
	const char* const RobotName = detectModel();
#else // have to use a string because aperios is annoying like that
	const std::string RobotName = detectModel();
#endif
	
	
	Capabilities::Capabilities(const char* robName, size_t numFrame, const char * const frameNames[], size_t numBut, const char * const butNames[], size_t numSen, const char * const senNames[], size_t pidOff, size_t numPID, size_t ledOff, size_t numLED, size_t numTotalOutputs)
		: name(robName),
		frames(frameNames,frameNames+numFrame), buttons(butNames,butNames+numBut), sensors(senNames,senNames+numSen),
		frameToIndex(), buttonToIndex(), sensorToIndex(),
		pidJointOffset(pidOff), numPIDJoints(numPID), ledOffset(ledOff), numLEDs(numLED), numOutputs(numTotalOutputs),
		fakeOutputs()
	{
		for(size_t i=0; i<frames.size(); ++i)
			frameToIndex[frames[i]]=i;
		for(size_t i=0; i<buttons.size(); ++i)
			buttonToIndex[buttons[i]]=i;
		for(size_t i=0; i<sensors.size(); ++i)
			sensorToIndex[sensors[i]]=i;
		
		std::map<std::string, const Capabilities*>::const_iterator it=getCaps().find(robName);
		if(it!=getCaps().end())
			std::cerr << "WARNING: RobotInfo '" << robName << "' capabilities has already been registered!  Name conflict?  Replacing previous..." << std::endl;
		getCaps()[robName]=this;
	}

	std::map<std::string, const Capabilities*>& Capabilities::getCaps() {
		static std::map<std::string, const Capabilities*> caps;
		return caps;
	}
	
}
