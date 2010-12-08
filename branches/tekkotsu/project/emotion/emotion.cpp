#include "emotion.h"
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <stdlib.h>


	void Emotion::move(int xoffset, int yoffset){
		x += xoffset; 
		y += yoffset;
		if (x > MAXVAL) x = MAXVAL;
		else if (x < MINVAL) x = MINVAL;
		if (y > MAXVAL) y = MAXVAL;
		else if (y < MINVAL) y = MINVAL;
		getstate();
		updatepose();
	}

	void Emotion::setpos(int xcoord, int ycoord){
		x = xcoord; y = ycoord;
		if (x > MAXVAL) x = MAXVAL;
		else if (x < MINVAL) x = MINVAL;
		if (y > MAXVAL) y = MAXVAL;
		else if (y < MINVAL) y = MINVAL;
		getstate();
		updatepose();
	}

	void Emotion::writeposefile(){
		fstream p(posefl,ios::out);
		p<<"#"<<estate<<endl;
		p<<"cd /usr/local/Tekkotsu/tools/dynamixel_util/"<<endl;
		for (int i = 0; i < MAXSERVOS; i++){
			if (currpose->getservo(i) != -1)
				p<<"./dynamixel_util move "<<i+1<<" "<<currpose->getservo(i)<<endl;
//"/usr/local/Tekkotsu/tools/dynamixel_util"
		}
		cout<<estate<<endl;
		p.close();
	}

	void Emotion::useposefile(){
		char cmnd[64] = "./";
		strcat(cmnd, posefl);
		system(cmnd);
		//cout<<cmnd<<endl;
	}

	void Emotion::getstate(){
		//char retval[128];retval[0] = '\0';
		estate[0] = '\0';
		char buffer[128]; int n;
		int exactstate = -1;
		bool exactflag = false;
		for (int i = 0; (!exactflag) && (i < STATECOUNT); i++){
			dist[i] = distance(i);
			if (dist[i] == 0){ 
				exactflag = true;
				exactstate = i;
			}
			//cout<<"Distance to #"<<i<<": "<<dist[i]<<endl;
		}
		if (exactflag){
			for (int i = 0; i < MAXCONTRIBUTION; i++)
				contribution[i] = 0;
			contribution[0] = 1; top[0] = exactstate;
			strcat(estate, state[exactstate]);
			strcat(estate, "-100%");
		} else{
			gettopx();
			//for (int i = 0; i < 3; i++) cout<<top[i]<<endl;
			setcontributions();
			for (int i = 0; i < MAXCONTRIBUTION; i++){
				strcat(estate, state[top[i]]);
				strcat(estate, "-");
				n = sprintf(buffer, "%.2f%% ", contribution[i] * 100);
				strcat(estate, buffer);
			}
		}
	}

	void Emotion::setcontributions(){
		double d[MAXCONTRIBUTION]; double totald = 0.0;
		double A[MAXCONTRIBUTION]; double Ainv[MAXCONTRIBUTION]; double totalAinv = 0.0;
		for (int i = 0; i < MAXCONTRIBUTION; i++){
			d[i] = dist[top[i]];
			totald += d[i];
		}
		for (int i = 0; i < MAXCONTRIBUTION; i++){
			A[i] = d[i] / totald;
			Ainv[i] = 1 / A[i];
			totalAinv += Ainv[i];
		}
		for (int i = 0; i < MAXCONTRIBUTION; i++){
			contribution[i] = Ainv[i] / totalAinv;
		}
	}

	void Emotion::updatepose(){
		//combine pose values here
		Pose *temp_pose[MAXCONTRIBUTION];
		//temp_pose = new Pose[MAXCONTRIBUTION];
		int tempint[MAXSERVOS]; int tempservoval; int counter = 1;
		double val;
		currpose->setname(estate);
		for (int i = 0; i < MAXSERVOS; i++) tempint[i] = -1;
		if (contribution[0] == 1){
			temp_pose[0] = pb->getpose(top[0]);
			for (int i = 0; i < MAXSERVOS; i++)
			 	currpose->setservo(i, temp_pose[0]->getservo(i));
		} else {
			for (int i = 0; i < MAXCONTRIBUTION; i++)
				temp_pose[i] = pb->getpose(top[i]);
			for (int i = 0; i < MAXSERVOS; i++){
				for (int j = 0; j < MAXCONTRIBUTION; j++){
					tempservoval = temp_pose[j]->getservo(i);
					if (tempservoval != -1){
						val = (tempservoval - 512) * contribution[j];
						if (tempint[i] == -1)
							tempint[i] = (int) val + 512;
						else {
							tempint[i] += (int) val;
						}
					}
				}
			}
			for (int i = 0; i < MAXSERVOS; i++)
				currpose->setservo(i, tempint[i]);
		}
	}

	/*char *Emotion::dtos(double param){
		char retval[32]; float temp = (float) param * 100;
		int n = sprintf(retval, "%.2f", temp);
		return strdup(retval);
	} */

	void Emotion::gettopx(){
		//only using MAXCONTRIBUTION # of values
		bool flag = false; int place;
		int indices[STATECOUNT];
		double temp; int swapval;
		for (int i = 0; i < STATECOUNT; i++) indices[i] = i;
		for (int i = 0; i < MAXCONTRIBUTION; i++){
			temp = dist[indices[i]]; place = -1; top[i] = indices[i];
			for (int j = i + 1; j < STATECOUNT; j++){
				if (dist[indices[j]] <= temp){
					temp = dist[indices[j]];
					place = indices[j];
				}
			}
			if (place > -1){
				swapval = indices[i];
				indices[i] = indices[place];
				indices[place] = swapval;
				top[i] = indices[i];
			}
		}
	}

	double Emotion::distance(int spot){
		double retval;
		double xdist; double ydist;
		if ((spot < 0) || (spot >= STATECOUNT)){
			cerr<<"Invalid state reference."<<endl;
		} else{
			xdist = xvals[spot] - x;
			ydist = yvals[spot] - y;
			xdist *= xdist; ydist *= ydist;
			retval = sqrt(xdist + ydist);
		}
		return retval;
	}
