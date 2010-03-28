//-*-c++-*-
#ifndef INCLUDED_NelsonInfo_h
#define INCLUDED_NelsonInfo_h

#include <cmath>
#include <stdlib.h>
#include "CommonInfo.h"
using namespace RobotInfo;

// see http://tekkotsu.org/porting.html#configuration for more information on TGT_HAS_* flags
#if defined(TGT_NELSON)
#  define TGT_IS_NELSON
#  define TGT_IS_BIOLOID
#  define TGT_IS_CREATE
#  define TGT_HAS_WEBCAM 1
#  define TGT_HAS_WHEELS 2
#  define TGT_HAS_BUTTONS 15
#  define TGT_HAS_LEDS 4
#  define TGT_HAS_CAMERA 1
#  define TGT_HAS_3D_ARMS 2
#  define TGT_HAS_HEAD 1
#endif

//! Contains information about an iRobot Create, such as number of joints, LEDs, etc.
namespace NelsonInfo {
	
	// *******************************
	//       ROBOT CONFIGURATION
	// *******************************

	extern const char* const TargetName; //!< the name of the model, to be used for logging and remote GUIs

	const unsigned int FrameTime=15;        //!< time between frames in the motion system (milliseconds)
	const unsigned int NumFrames=1;        //!< the number of frames per buffer (don't forget also double buffered)
	const unsigned int SoundBufferTime=32; //!< the number of milliseconds per sound buffer... I'm not sure if this can be changed
	
	//!@name Output Types Information
	const unsigned NumWheels      =  2;
	
	const unsigned JointsPerArm   =  4;
	const unsigned NumArms        =  2;
	const unsigned NumArmJoints   =  JointsPerArm*NumArms;
	
	const unsigned JointsPerLeg   =  0; //!< The number of joints per leg
	const unsigned NumLegs        =  0; //!< The number of legs
	const unsigned NumLegJoints   =  JointsPerLeg*NumLegs; //!< the TOTAL number of joints on ALL legs
	const unsigned NumHeadJoints  =  11; //!< The number of joints in the neck & FACE
	const unsigned NumTailJoints  =  0; //!< The number of joints assigned to the tail
	const unsigned NumMouthJoints =  0; //!< the number of joints that control the mouth
	const unsigned NumEarJoints   =  0; //!< The number of joints which control the ears (NOT per ear, is total)
	const unsigned NumButtons     =  15; //!< the number of buttons that are available
	const unsigned NumSensors     =  19;  //!< the number of sensors available
	const unsigned NumLEDs        =  9; //!< The number of LEDs which can be controlled (3 neck servos, 2 virtual LEDs)
	const unsigned NumFacePanelLEDs = 0; //!< The number of face panel LEDs
	
	const unsigned NumPIDJoints   = NumWheels + NumArmJoints + NumHeadJoints; //!< servo pins
	const unsigned NumOutputs     = NumPIDJoints + NumLEDs; //!< the total number of outputs
	const unsigned NumReferenceFrames = NumOutputs + 1 + NumArms; // + 1; //!< for the base, gripper (* NumArms), and Camera

	// TODO: double check figures
	// webcam ?
	const float CameraHorizFOV=60*(float)M_PI/180; //!< horizontal field of view (radians)
	const float CameraVertFOV=45*(float)M_PI/180; //!< vertical field of view (radians)
	const float CameraFOV=CameraHorizFOV; //!< should be set to maximum of #CameraHorizFOV or #CameraVertFOV
	const unsigned int CameraResolutionX=320; //!< the number of pixels available in the 'full' layer
	const unsigned int CameraResolutionY=240; //!< the number of pixels available in the 'full' layer
	//@}


	// *******************************
	//         OUTPUT OFFSETS
	// *******************************

	//!Corresponds to entries in ERS7Info::PrimitiveName, defined at the end of this file
	//!@name Output Offsets
	
	const unsigned PIDJointOffset = 0; //!< The beginning of the PID Joints
	const unsigned WheelOffset = PIDJointOffset;
    const unsigned ArmOffset = WheelOffset+NumWheels;
    const unsigned LeftArmOffset = ArmOffset;
    const unsigned RightArmOffset = ArmOffset + JointsPerArm;
	const unsigned HeadOffset  = ArmOffset+NumArmJoints;   //!< the offset of the beginning of the head joints, add TPROffset_t to get specific joint

	const unsigned LEDOffset   = PIDJointOffset + NumPIDJoints;
	
	const unsigned ModeOffset = LEDOffset + NumLEDs;

	const unsigned BaseFrameOffset   = NumOutputs; //!< Use with kinematics to refer to base reference frame
	const unsigned CameraFrameOffset = BaseFrameOffset + 1; //!< Use with kinematics to refer to camera reference frame

	//! The offsets of appendages with pan (heading), tilt (elevation), note that this should be added to HeadOffset, otherwise use HeadOffset_t (#HeadPanOffset and #HeadTiltOffset)
	//enum TPROffset_t {
	//	PanOffset = 0,      //!< pan/heading (horizontal)
	//	TiltOffset, //!< tilt/elevation (vertical)
	//	NodOffset = TiltOffset //!< replicated tilt (could be left undefined instead...)
	//};    3/28 MEF
    enum TPROffset_t {
		TiltOffset = 0, //!< tilt/elevation (vertical)
		PanOffset,      //!< pan/heading (horizontal)
		RollOffset,      //!< roll (rotational)
		NodOffset=RollOffset       //!< nod (second tilt)
	};
	
    enum ArmOffset_t {      // 3/28/10 MEF
        ArmShoulderPitchOffset=ArmOffset,
        ArmShoulderOffset,
        ArmElbowOffset,
        GripperOffset
    };

	//! These are 'absolute' offsets for the neck joints, don't need to add to HeadOffset like TPROffset_t values do
	enum HeadOffset_t {
		HeadPanOffset = HeadOffset,      //!< pan/heading (horizontal)
		HeadTiltOffset, //!< tilt/elevation (vertical)
        HeadNodOffset,
        EyeTiltOffset,
        EyePanLeftOffset,
        EyePanRightOffset,
        EyeLidOffset,
        TopLipOffset,
        BottomLipOffset,
        LeftEyebrowOffset,
        RightEyebrowOffset
	};

	enum WheelOffset_t {
		LWheelOffset=WheelOffset,
		RWheelOffset
	};
	
	//! The offsets of the individual LEDs
	/*! @hideinitializer */
	enum LEDOffset_t {
		PowerRedLEDOffset=LEDOffset,
		PowerGreenLEDOffset,
		PlayLEDOffset,
		AdvanceLEDOffset,
        PanLEDOffset,           // 3/28/10 MEF
        TiltLEDOffset,
        NodLEDOffset,
        LeftCheekLEDOffset,
        RightCheekLEDOffset 
	};

	const LEDOffset_t RedLEDOffset = PowerRedLEDOffset;
	const LEDOffset_t BlueLEDOffset = AdvanceLEDOffset; //!< Create has no blue LED: use Advance LED here, and Advance+PowerRED in BlueLEDMask
	const LEDOffset_t GreenLEDOffset = PlayLEDOffset;
	const LEDOffset_t YellowLEDOffset = AdvanceLEDOffset;

	typedef unsigned int LEDBitMask_t; //!< So you can be clear when you're refering to a LED bitmask

	const LEDBitMask_t BlueLEDMask = (1<<(AdvanceLEDOffset-LEDOffset)) |
					 (1<<(PowerRedLEDOffset-LEDOffset)); //!< Create has no blue LED, so use Advance (green) + Power (red)
	const LEDBitMask_t GreenLEDMask = 1<<(GreenLEDOffset-LEDOffset); //!< mask corresponding to GreenLEDOffset
	const LEDBitMask_t YellowLEDMask = 1<<(YellowLEDOffset-LEDOffset); //!< mask corresponding to YellowLEDOffset
	const LEDBitMask_t RedLEDMask = 1<<(RedLEDOffset-LEDOffset); //!< mask corresponding to RedLEDOffset

	const LEDBitMask_t PowerRedLEDMask = 1<<(PowerRedLEDOffset-LEDOffset); //!< mask corresponding to BlueLEDOffset
	const LEDBitMask_t PowerGreenLEDMask = 1<<(PowerGreenLEDOffset-LEDOffset); //!< mask corresponding to GreenLEDOffset
	const LEDBitMask_t PlayLEDMask = 1<<(PlayLEDOffset-LEDOffset); //!< mask corresponding to YellowLEDOffset
	const LEDBitMask_t AdvanceLEDMask = 1<<(AdvanceLEDOffset-LEDOffset); //!< mask corresponding to RedLEDOffset

    const LEDBitMask_t PanLEDMask = 1<<(PanLEDOffset-LEDOffset);  // 3/28/10 MEF
    const LEDBitMask_t TiltLEDMask = 1<<(TiltLEDOffset-LEDOffset);
    const LEDBitMask_t NodLEDMask = 1<<(NodLEDOffset-LEDOffset);
    const LEDBitMask_t LeftCheekLEDMask = 1<<(LeftCheekLEDOffset-LEDOffset);
    const LEDBitMask_t RightCheekLEDMask = 1<<(RightCheekLEDOffset-LEDOffset);

	//! LEDs for the face panel (all FaceLEDPanelMask<<(0:NumFacePanelLEDs-1) entries)
	const LEDBitMask_t FaceLEDMask = 0;
	//! selects all of the leds
	const LEDBitMask_t AllLEDMask  = (LEDBitMask_t)~0;
	//@}

	// *******************************
	//          INPUT OFFSETS
	// *******************************


	//! The order in which inputs should be stored
	//!@name Input Offsets

	//! holds offsets to different buttons in WorldState::buttons[]
	/*! Should be a straight mapping to the ButtonSourceIDs
	 *
	 *  @see WorldState::buttons
	 * @hideinitializer */
	enum ButtonOffset_t {
		PlayButOffset, //!< 1 if play button is down
		AdvanceButOffset, //!< 1 if advance button is down
		WallButOffset, //!< 1 if wall is detected (note correspondence to WALL_SIGNAL_OFFSET's value, avoiding problems if the two are swapped)
		DropCasterButOffset, //!< 1 if caster detects dropoff
		DropLeftWheelButOffset, //!< 1 if left wheel detects dropoff
		DropRightWheelButOffset, //!< 1 if right wheel detects dropoff
		BumpLeftButOffset, //!< 1 if left bumper is pressed
		BumpRightButOffset, //!< 1 if right bumper is pressed
		OvercurrentLeftWheelOffset, //!< 1 if the left wheel is drawing more than 1 amp
		OvercurrentRightWheelOffset, //!< 1 if the right wheel is drawing more than 1 amp
		LowSideDriver0ButOffset, //!< 1 if low side driver 0 is pulling more than 0.5 amps
		LowSideDriver1ButOffset, //!< 1 if low side driver 1 is pulling more than 0.5 amps
		LowSideDriver2ButOffset, //!< 1 if low side driver 2 is pulling more than 1.6 amps
		BaseChargerButOffset, //!< 1 if the home base charger is available
		InternalChargerButOffset //!< 1 if the internal charger is available
	};

	const ButtonOffset_t GreenButOffset = PlayButOffset;
	const ButtonOffset_t YellowButOffset = AdvanceButOffset;
	const ButtonOffset_t RedButOffset = PlayButOffset;

	//! Provides a string name for each button
	const char* const buttonNames[NumButtons+1] = { 
		"Play", "Advance",
		"Wall",
		"CasterDrop", "LWheelDrop", "RWheelDrop",
		"LBump", "RBump",
		"LOverCurrent", "ROverCurrent",
		"DriverLow0", "DriverLow1", "DriverLow2",
		"BaseCharger", "InternalCharger",
		NULL
	};

	//! holds offset to different sensor values in WorldState::sensors[]
	/*! @see WorldState::sensors[] */
	enum SensorOffset_t {
		DigitalInput0Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput1Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput2Offset, //!< the digital input pins in bits 0 through 4
		DigitalInput3Offset, //!< the digital input pins in bits 0 through 4
		AnalogSignalOffset, //!< voltage on cargo bay pin 4
		WallSignalOffset, //!< strength of the wall sensor's signal (note correspondence to WALL_OFFSET's value, avoid problems if the two are swapped)
		IRCommOffset, //!< value received by the infrared communication receiver, see IRComm_t for values sent by standard hardware
		CliffLeftSignalOffset, //!< strength of the left cliff sensor
		CliffFrontLeftSignalOffset, //!< strength of the front left cliff sensor
		CliffFrontRightSignalOffset, //!< strength of the front right cliff sensor
		CliffRightSignalOffset, //!< strength of the right cliff sensor
		EncoderDistanceOffset, //!< average distance (mm) traveled by the wheels since last update
		EncoderAngleOffset, //!< average angle (radians) rotated since the last update
		VoltageOffset, //!< mV measured at battery
		CurrentOffset, //!< mA flowing into battery (negative when discharging)
		BatteryChargeOffset, //!< mAh remaining in battery (may not be accurate with alkaline battery pack)
		BatteryTempOffset, //!< degrees celsius
		ChargingStateOffset, //!< one of #ChargingState_t
		ModeStateOffset //!< one of #ModeState_t
	};
	
	enum IRComm_t {
		IR_REMOTE_LEFT=129,
		IR_REMOTE_FORWARD,
		IR_REMOTE_RIGHT,
		IR_REMOTE_SPOT,
		IR_REMOTE_MAX,
		IR_REMOTE_SMALL,
		IR_REMOTE_MEDIUM,
		IR_REMOTE_LARGE,
		IR_REMOTE_PAUSE,
		IR_REMOTE_POWER,
		IR_REMOTE_ARC_LEFT, 
		IR_REMOTE_ARC_RIGHT,
		IR_REMOTE_STOP,
		IR_REMOTE_SEND,
		IR_REMOTE_DOCK,
		IR_BASE_RED=248,
		IR_BASE_GREEN=244,
		IR_BASE_FORCE=242,
		IR_BASE_RED_GREEN=252,
		IR_BASE_RED_FORCE=250,
		IR_BASE_GREEN_FORCE=246,
		IR_BASE_RED_GREEN_FORCE=254
	};
	/*const unsigned IR_BASE_MASK=240;
	const unsigned IR_BASE_RED_MASK=8;
	const unsigned IR_BASE_GREEN_MASK=4;
	const unsigned IR_BASE_FORCE_MASK=2;*/
	
	enum ChargingState_t {
		CHARGING_OFF,
		CHARGING_RECONDITIONING,
		CHARGING_FULL,
		CHARGING_TRICKLE,
		CHARGING_WAITING,
		CHARGING_FAULT
	};

	enum ModeState_t {
		MODE_OFF,
		MODE_PASSIVE,
		MODE_SAFE,
		MODE_FULL
	};
		
	//! Provides a string name for each sensor
	const char* const sensorNames[NumSensors+1] = { 
		"DigitalIn0",
		"DigitalIn1",
		"DigitalIn2",
		"DigitalIn3",
		"AnalogIn",
		"WallSignal",
		"IR",
		"CliffLeftSignal",
		"CliffFrontLeftSignal",
		"CliffFrontRightSignal",
		"CliffRight",
		"Distance",
		"Angle",
		"BatteryVoltage",
		"BatteryCurrent",
		"BatteryCharge",
		"BatteryTemp",
		"ChargingState",
		"ModeState",
		NULL
	};

	//@}


	//! Names for each of the outputs
	const char* const outputNames[NumReferenceFrames+1] = {
		"WHEEL:L",
		"WHEEL:R",
        "ARM_L:pitch","ARM_L:shldr","ARM_L:elbow","ARM_L:gripper",
        "ARM_R:pitch","ARM_R:shldr","ARM_R:elbow","ARM_R:gripper",
        "NECK:pan","NECK:tilt","NECK:nod",
        "HEAD:eyes","HEAD:leye","HEAD:reye","HEAD:eyel",
        "HEAD:tlip","HEAD:blip","HEAD:lbrow","HEAD:rbrow",
		"LED:Power(Red)",
		"LED:Power(Green)",
		"LED:Play",
		"LED:Advance",
        "LED:Pan","LED:Tilt","LED:Nod","LED:lcheek","LED:rcheek",
		"DesiredMode",
		"BaseFrame",
		"CameraFrame",
		NULL
	};
	
	//! allocation declared in RobotInfo.cc
	extern const Capabilities capabilities;

    //! offsets into DefaultPIDs, since Dynamixel servos don't actually use PID control, but a different set of parameters
	enum ServoParam_t {
		DYNAMIXEL_SLOPE=0, //!< compliance slope, the proportional control (P in PID)
		DYNAMIXEL_PUNCH, //!< punch, a constant added to the slope once error exceeds compliance margin
		DYNAMIXEL_MARGIN //!< compliance margin, the amount of error to tolerate before triggering a torque response
	};

	//! This table holds the default PID values for each joint.  see PIDMC
	const float DefaultPIDs[NumPIDJoints][3] = {
      {1,0,0},
	  {1,0,0},  
      {32,32,0},{32,32,0},{32,32,0},{32,32,0},
      {32,32,0},{32,32,0},{32,32,0},{32,32,0},
      {32,32,0},{32,32,0},{32,32,0},
      {32,32,0},{32,32,0},{32,32,0},{32,32,0},
      {32,32,0},{32,32,0},{32,32,0},{32,32,0},
	};
	
	//!These values are our recommended maximum joint velocities, in rad/ms
	const float MaxOutputSpeed[NumOutputs] = {
        // wheels
		0,0,
        // servos
		0,0,0,0,
		0,0,0,0,
        0,0,0,
        0,0,0,0,
        0,0,0,0,
        // leds
        0,
        0,
        0,
        0,
		0,0,0,0,
		0
	};

	#ifndef RAD
		//!Just a little macro for converting degrees to radians
	#define RAD(deg) (((deg) * (float)M_PI ) / 180.0f)
		//!a flag so we undef these after we're done - do you have a cleaner solution?
	#define __RI_RAD_FLAG
	#endif
	
	//! This table holds the software limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	const float outputRanges[NumOutputs][2] =
		{
			{ -1 , 1 },
			{ -1 , 1 },
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },            
			{  0 , 1 },{  0 , 1 },{  0 , 1 },{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};

	//! This table holds the mechanical limits of each of the outputs, first index is the output offset, second index is MinMaxRange_t (i.e. MinRange or MaxRange)
	/*! Same as #outputRanges, don't know actual values because they were never specified by Sony */
	const float mechanicalLimits[NumOutputs][2] =
		{
			{ -1 , 1 },
			{ -1 , 1 },
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
            {RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},{RAD(-100),RAD(100)},
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },
			{  0 , 1 },            
			{  0 , 1 },{  0 , 1 },{  0 , 1 },{  0 , 1 },
			{ MODE_PASSIVE, MODE_SAFE }
		};

#ifdef __RI_RAD_FLAG
#undef RAD
#undef __RI_RAD_FLAG
#endif
}

/*! @file
 * @brief Defines some capabilities of the iRobot Create robots
 * @author ejt (Creator)
 *
 * $Author: dst $
 * $Name:  $
 * $Revision: 1.33 $
 * $State: Exp $
 * $Date: 2009/12/24 00:02:52 $
 */

#endif
