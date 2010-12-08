/*
 *  ofxFiducial.h
 *  openFrameworks
 *
 *  Created by Alain Ramos a.k.a. ding
 *  Copyright 2008 Alain Ramos.
 *
 *
 For Free you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 */
//	----------------------------------------------------------------------------------------------------

#ifndef VISION_BLOCK_FIDUCIAL_H
#define VISION_BLOCK_FIDUCIAL_H

#include "libfidtrack/fidtrackX.h"

#define INVALID_ID     (-1)

#define FIDUCIAL_LOST 0
#define FIDUCIAL_FOUND 1
#define FIDUCIAL_INVALID 2
#define FIDUCIAL_WRONG 3
#define FIDUCIAL_REGION 4

#define PI              (3.1415)
#define TWO_PI          (2*PI)


//this is for storing x, y and angle 
//and to figure out speed and acceleration of position, rotation and angle
struct _frame {
 float xpos,ypos,angle;
 //float rotation_speed, rotation_accel;
 //float motion_speed, motion_accel;
 //float motion_speed_x, motion_speed_y;
 int time;
 //float cameraToScreen_xpos,cameraToScreen_ypos;
 };

class Fiducial {
	
public:
	//public variables
	//****//--------------------------------------------------------------------------------------
	int		fidId; // fiducial id
	float	r_size, l_size; //root size and leaf size
	_frame	current, last; //current and last fid info
	//bool	updateCorners; //do we want to auto update corners?
	//vector	<ofPoint> cornerPoints;//vector to store corner points
	
	//Constructor
	//****//--------------------------------------------------------------------------------------
	Fiducial() {
		fidId			= INVALID_ID;
		current.xpos	= current.ypos = -100.0f;
		current.angle	= 0.0f;
		//current.rotation_speed	= current.rotation_accel	= 0.0f;
		//current.motion_speed	= current.motion_accel		= 0.0f;
		//current.motion_speed_x	= current.motion_speed_y	= 0.0f; 
		alive			= true;
		//cornersUpdated  = false;
		//updated			= false;
		//updateCorners	= false;
		life			= 3;
		current.time	= 0;
		//cornerPoints.resize(4);
		saveLastFrame();
	}
	
	//getter functions
	//****//--------------------------------------------------------------------------------------
	int	  getId()		{ return fidId; }
	//float getMSpeed()	{ return current.motion_speed; }
	//float getMAccel()	{ return current.motion_accel; }
	//float getMSpeedX()	{ return current.motion_speed_x; }
	//float getMSpeedY()	{ return current.motion_speed_y; }
	float getX()		{ return current.xpos; }
	float getY()		{ return current.ypos; }
	float getAngle()	{ return TWO_PI - current.angle; } //radian (phi)
	float getAngleDeg()	{ return 360 - (current.angle * ( 180 / PI )); } //reversed to compensate for OF
	//float getRSpeed()	{ return current.rotation_speed; }
	//float getRAccel()	{ return current.rotation_accel; }
	float getRootSize()	{ return r_size; }
	//bool  getCornerUpdateStatus() { return updateCorners; }
	//void  setUpdateCorners(bool _update){ updateCorners	= _update; }

    
	void initLastFrame(){
		//printf("fiducial.initLastFrame:%p:\n", this);
		saveLastFrame();
	}

	//------------------------
	//Update Fiducial
	//****//--------------------------------------------------------------------------------------
	void update(float _x, float _y, float _angle, float _root, float _leaf) {
			//printf("---fiducial.update:%p:\n", this);
			//printf("fiducial.update.raw:%f:%f\n", _x,_y);
		//this is to try and filter out some of the jitter
		//------------------------------------------------
		float jitterThreshold	= 1.0;
		
		//if new posit - current posit is less than threshold dont update it must be jitter 
		if ( fabs(_x - current.xpos) > jitterThreshold ) current.xpos = _x;
		else current.xpos = last.xpos;
		if ( fabs(_y - current.ypos) > jitterThreshold ) current.ypos = _y;
		else current.ypos = last.ypos;
		//if new angle - current angle is less than threshold/20 dont update it must be jitter
		if ( fabs(_angle - current.angle) > jitterThreshold/20 ) current.angle = _angle;
		else current.angle = last.angle;
		//------------------------------------------------
		//printf("fiducial.update:%f:%f\n", current.xpos,current.ypos);
		current.time	= 0; //ofGetElapsedTimeMillis(); //get current time
		r_size			= _root; //update root size
		l_size			= _leaf; //update leaf size
		state			= FIDUCIAL_FOUND; //fiducial found
		//updated			= true; //got updated
		
		//computeSpeedAccel(); //compute speed & acceleration
		
		//if ( updateCorners ) computeCorners(); //figures out the corners and fills the cornerPoints vector
		//else cornersUpdated = false;
		saveLastFrame(); //last frame equal to this frame
	}
	
	//Operator overloading for FiducialX & ofxFiducial
	//****//--------------------------------------------------------------------------------------
	void operator=( const FiducialX& fiducial) {
		fidId			= fiducial.id;
		current.xpos	= fiducial.x;
		current.ypos	= fiducial.y;
		current.angle	= fiducial.angle;
		r_size			= fiducial.root_size;
		l_size			= fiducial.leaf_size;
		current.time	= 0; //ofGetElapsedTimeMillis();
	}
	
	void operator=( const Fiducial& fiducial) {
		fidId			= fiducial.fidId;
		current.xpos	= fiducial.current.xpos;
		current.ypos	= fiducial.current.ypos;
		current.angle	= fiducial.current.angle;
		r_size			= fiducial.r_size;
		l_size			= fiducial.l_size;
		current.time	= fiducial.current.time;
	}
	
	//private stuff
	//****//--------------------------------------------------------------------------------------
private:
	bool	alive; //can we remove fid from list
	int		life;//fids lifespan in frames
	int		state;//fids lost/found state
	
	// conversions from degree to radian and back
	float radians (float degrees) {
		return degrees*PI/180.0;
	}
	
	float degrees (float radians) {
		return radians*180.0/PI;
	}	

	//make last frame = this frame
	void saveLastFrame() {
		last.time	= current.time;
		last.xpos	= current.xpos;
		last.ypos	= current.ypos;
		last.angle	= current.angle;
		//last.motion_speed	= current.motion_speed;
		//last.rotation_speed = current.rotation_speed;
	}
};

#endif
