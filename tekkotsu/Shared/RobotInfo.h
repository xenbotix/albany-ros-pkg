//-*-c++-*-
#ifndef INCLUDED_RobotInfo_h
#define INCLUDED_RobotInfo_h

#include <map>
#include <string>

// If creating a new robot configuration, add a new entry in the list below

#if TGT_NELSON
#	include "Shared/NelsonInfo.h"
namespace RobotInfo { using namespace NelsonInfo; }

#elif TGT_ERS210
#	include "Shared/ERS210Info.h"
namespace RobotInfo { using namespace ERS210Info; }

#elif TGT_ERS220
#	include "Shared/ERS220Info.h"
namespace RobotInfo { using namespace ERS220Info; }

#elif TGT_ERS2xx
#	include "Shared/ERS2xxInfo.h"
namespace RobotInfo { using namespace ERS2xxInfo; }

#elif TGT_ERS7
#	include "Shared/ERS7Info.h"
namespace RobotInfo { using namespace ERS7Info; }

#elif TGT_LYNXARM6
#	include "Shared/LynxArm6Info.h"
namespace RobotInfo { using namespace LynxArm6Info; }

#elif TGT_REGIS1
#	include "Shared/Regis1Info.h"
namespace RobotInfo { using namespace Regis1Info; }

#elif TGT_QBOTPLUS
#	include "Shared/QBotPlusInfo.h"
namespace RobotInfo { using namespace QBotPlusInfo; }

#elif TGT_QWERK
#	include "Shared/QwerkInfo.h"
namespace RobotInfo { using namespace QwerkInfo; }

#elif TGT_WIIMOTE
#	include "Shared/WiiMoteInfo.h"
namespace RobotInfo { using namespace WiiMoteInfo; }

#elif TGT_CREATE
#	include "Shared/CreateInfo.h"
namespace RobotInfo { using namespace CreateInfo; }

#elif TGT_CALLIOPE
#	include "Shared/CalliopeInfo.h"
namespace RobotInfo { using namespace CalliopeInfo; }

#elif TGT_BIOLOID
#	include "Shared/BioloidInfo.h"
namespace RobotInfo { using namespace BioloidInfo; }

#elif TGT_PANTILT
#	include "Shared/PanTiltInfo.h"
namespace RobotInfo { using namespace PanTiltInfo; }

#elif TGT_HANDEYE
#	include "Shared/HandEyeInfo.h"
namespace RobotInfo { using namespace HandEyeInfo; }

#elif TGT_HANDEYEZ
#	include "Shared/HandEyeZInfo.h"
namespace RobotInfo { using namespace HandEyeZInfo; }

#elif TGT_CHIARA
#	include "Shared/ChiaraInfo.h"
namespace RobotInfo { using namespace ChiaraInfo; }

#elif TGT_DYNAMIC
#	include "Shared/DynamicInfo.h"
namespace RobotInfo { using namespace DynamicInfo; }

#elif TGT_TENTACLE
#	include "Shared/TentacleInfo.h"
namespace RobotInfo { using namespace TentacleInfo; }

#else //default case, currently ERS-7
#	warning "TGT_<model> undefined or unknown model set - defaulting to ERS7"
#	include "Shared/ERS7Info.h"
namespace RobotInfo { using namespace ERS7Info; }
#endif //model selection


//! Contains information about the robot, such as number of joints, PID defaults, timing information, etc.
/*! This is just a wrapper for whichever namespace corresponds to the current
*  robot target setting (one of TGT_ERS7, TGT_ERS210, TGT_ERS220, or the cross-booting TGT_ERS2xx)
*
*  You probably should look at ERS7Info, ERS210Info, ERS220Info, or ERS2xxInfo for the actual
*  constants used for each model, although some common information shared by all of these namespaces
*  is defined in CommonInfo.h */
namespace RobotInfo {
	
	//! Accessor for Capabilities::caps, returns the Capabilities instance for a specified robot model (or NULL if robot is unknown or didn't provide a Capabilities instance)
	/*! Use this if you have a robot name in string form and want to check or map its capabilities.
	 *  (For example, if you are communicating with another robot of a different type over the network.)
	 *  If you know at compile time the type of the robot in question, you could just directly access
	 *  its 'capabilities' instance via its RobotInfo namespace.  (e.g. ERS210Info::capabilities)
	 *  If you want the capabilities for the current robot, just use the global 'capabilities' instance
	 *  as RobotInfo.h will automatically import the current robot's namespace into the global space. */
	inline const Capabilities* getCapabilities(const std::string& robName) {
		const std::map<std::string, const Capabilities*>& caps = Capabilities::getCaps();
		std::map<std::string, const Capabilities*>::const_iterator it = caps.find(robName);
		return it==caps.end() ? NULL : it->second;
	}
	
}

using namespace RobotInfo;
	
/*! @file
 * @brief Checks the define's to load the appropriate header and namespace
 * @author ejt (Creator)
 *
 * $Author: dst $
 * $Name:  $
 * $Revision: 1.41 $
 * $State: Exp $
 * $Date: 2010/03/09 04:58:17 $
 */

#endif
