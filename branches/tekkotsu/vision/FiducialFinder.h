/*
 *  ofxFiducialFinder.h
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

#ifndef VISION_BLOCK_FIDUCIAL_FINDER_H
#define VISION_BLOCK_FIDUCIAL_FINDER_H

#include <math.h>
#include <cv.h>
#include "Fiducial.h"

#include <vector>
#include <string>

#include "libfidtrack/segment.h"

#define MAX_FIDUCIAL_COUNT 512

//*****************************************************************
//simple fiducial finder using a vector
//resets every frame and doesn't keep track of previous fiducials
//also doesnt get acceleration, speed, etc info

class FiducialFinder {
	
	public:
		//constructor & destructor
		FiducialFinder();
		~FiducialFinder();
	
		//vector to store fiducials
		std::vector <Fiducial> _fiducials;
	
		//find fiducials in gray image
		void findFiducials( IplImage& input ); // TODO
	
		//initialize the tree where the fiducial data is from file
		void initTree( const char *file_name );
	
		//initialize the default tree where the fiducial data is
		void initDefaultTree();
	
	private:
		//width and height of gray image
		int m_width, m_height;
		//is the segmenter initialized
		bool initialized;
		//is the tree initialized from file
		bool treeFromFile;
		//segmenter
		Segmenter segmenter;
		//tree ID map
		TreeIdMap treeidmap;

        unsigned char* pixels;
        unsigned char* t_pixels;
	
		//fiducial tracker
		FidtrackerX fidtrackerx;
		//max number of fiducials
		FiducialX fiducials[ MAX_FIDUCIAL_COUNT*2 ];
			
		//deinitialize segmenter
		void deinitSegmenter();
};

#endif
