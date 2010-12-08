#ifndef emotion_included
#define emotion_included
#include <iostream>
#include <math.h>
#include <string.h>
#include "poses.h"

#define MAXCONTRIBUTION 3
#define NEUTRAL 512

using namespace std;

        const int MINVAL = 0;
        const int MAXVAL = 100;
	const int STATECOUNT = 13;
        const char state[][128] = {"Useless", "Understimulated", "Content", "Apathetic",
                        "Patient", "Confused", "Neutral", "Informed", 
			"Impatient", "Curious", "Frustrated", "Overstimulated", 
			"Helpful"};
	const int xvals[] = {0, 50, 100, 30, 70, 10, 50, 90, 30, 70, 0, 50, 100};
	const int yvals[] = {0, 0, 0, 25, 25, 50, 50, 50, 75, 75, 100, 100, 100};
	const char posefl[] = "newpose";
	class Emotion{
		private:
	                int x, y;			
			int top[MAXCONTRIBUTION];
			char estate[256];
				//using smallest 3 distances
			char infilenm[128];
			double contribution[MAXCONTRIBUTION];  
			double dist[STATECOUNT];
			void getstate();
			void setcontributions();
			void updatepose();
			Pose *currpose;
			Posebank *pb;
                public:
			Emotion(){x = 50; y = 50; currpose = new Pose(); strcpy(infilenm, "nelsonposedata.txt"); 
				pb = new Posebank(infilenm); getstate(); updatepose();}
			void move(int xoffset, int yoffset);
			void setpos(int xcoord, int ycoord);
			char *currentstate(){return estate;}
			Pose *currentpose(){return currpose;}
			double distance(int spot);
			void gettopx();
			void writeposefile();
			void useposefile();
	};

#endif
