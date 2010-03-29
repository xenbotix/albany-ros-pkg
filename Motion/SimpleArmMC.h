//-*-c++-*-
#ifndef INCLUDED_SimpleArmMC_h
#define INCLUDED_SimpleArmMC_h

#include "Shared/RobotInfo.h"
#include "MotionCommand.h"
#include "OutputCmd.h"
#include "Shared/mathutils.h"

//! This class allows us to pose Nelson's face
class SimpleArmMC : public MotionCommand {
public:
  //! Constructor, defaults to all joints to current value in ::state (i.e. calls takeSnapshot() automatically)
  SimpleArmMC(int index);
	
  //! Destructor
  virtual ~SimpleArmMC() {}

  void freezeMotion();
  void takeSnapshot();

  //!@name Speed Control
	
  //! Sets #maxSpeed to 0 (no maximum)
  void noMaxSpeed() { for(unsigned int i=0; i<JointsPerArm; i++) maxSpeed[i]=0; }
	
  //! Restores #maxSpeed to default settings from Config::Motion_Config
  /*! @param x ratio of the max speed to use; so 0.5 would limit motion to half the recommended upper limit */
  void defaultMaxSpeed(float x=1);
	
  //! Sets #maxSpeed in rad/sec
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @param x maximum radians per second to move */
  void setMaxSpeed(unsigned int i, float x) { maxSpeed[i]=x*FrameTime/1000; }
	
  //! Returns #maxSpeed in rad/sec
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @return the maximum speed of joint @a i in radians per second */
  float getMaxSpeed(unsigned int i) { return maxSpeed[i]*1000/FrameTime; }
	
  //@}
	
  //!@name Joint Accessors
	
  //! Sets the weight values for all the neck joints
  void setWeight(float w);
	
  //! Request a set of neck or face joint values
  void setJoints(float pitch, float shoulder, float elbow, float grip);

  //! Directly set a single neck joint value
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t)
   *  @param value the value to be assigned to join @a i, in radians */
  void setJointValue(unsigned int i, float value) {
#ifdef TGT_HAS_HEAD
    if(!ensureValidJoint(i))
      return;
    armTargets[i]=clipAngularRange(ArmOffset+(JointsPerArm*ArmIndex)+i,value);
    markDirty();
#endif
  }
	
  //! Returns the target value of joint @a i.  Use this if you want to know the current @b commanded joint value; To get the current joint @b position, look in WorldState::outputs
  /*! @param i joint offset relative to HeadOffset (i.e. one of TPROffset_t) */
  float getJointValue(unsigned int i) const {
    if(ensureValidJoint(i))
      return armTargets[i];
    else
      return 0;
  }
	
  //@}
	
public:	
  //!@name Inherited:
  virtual int updateOutputs(); //!< Updates where the arm is
  virtual int isDirty() { return dirty; } //!< true if a change has been made since the last updateJointCmds() and we're active
  virtual int isAlive(); //!< Alive while target is not reached
  virtual void DoStart() { MotionCommand::DoStart(); markDirty(); } //!< marks this as dirty each time it is added
  //@}

protected:
  //! puts x in the range (-pi,pi)
  static float normalizeAngle(float x) { return x - static_cast<float>( rint(x/(2*M_PI)) * (2*M_PI) ); }
	
  //! if @a x is outside of the range of joint @a i, it is set to either the min or the max, whichever is closer
  static float clipAngularRange(unsigned int i, float x) {
    float min=outputRanges[i][MinRange];
    float max=outputRanges[i][MaxRange];
    if(x<min || x>max) {
      float mn_dist=std::abs(normalizeAngle(min-x));
      float mx_dist=std::abs(normalizeAngle(max-x));
      if(mn_dist<mx_dist)
	return min;
      else
	return max;
    } else
      return x;
  }
  //! if targetReached, reassigns headCmds from MotionManager::getOutputCmd(), then sets dirty to true and targetReached to false
  /*! should be called each time a joint value gets modified in case
   *  the head isn't where it's supposed to be, it won't jerk around
   * 
   *  MotionManager::getOutputCmd() is called instead of
   *  WorldState::outputs[] because if this is being called rapidly
   *  (i.e. after every sensor reading) using the sensor values will
   *  cause problems with very slow acceleration due to sensor lag
   *  continually resetting the current position.  Using the last
   *  value sent by the MotionManager fixes this.*/
  void markDirty();
  static bool ensureValidJoint(unsigned int& i);

  int ArmIndex;
  bool dirty;                          //!< true if a change has been made since last call to updateJointCmds()
  float armTargets[JointsPerArm];    //!< stores the target value of each joint
  OutputCmd armCmds[JointsPerArm];   //!< stores the last values we sent from updateOutputs
  float maxSpeed[JointsPerArm];       //!< initialized from Config::motion_config, but can be overridden by setMaxSpeed(); rad per frame
};

#endif
