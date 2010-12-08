#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "poses.h"

	Loader::Loader(char filename[128]){
		FILE *infile;
		char buffer[128]; buffer[0] = '\0';
		size= 0;
		for (int i = 0; i < MAXPOSES; i++)
			linein[i][0] = '\0';
		if ((infile = fopen(filename, "r")) == NULL){
			cerr<<"Error opening "<<filename<<endl;
			exit(1);
		}
		while (fscanf(infile, "%s", buffer) != EOF){
			//cout<<"TESTING "<<buffer<<endl;
			strcpy(linein[size], buffer);
			size++;
			buffer[0] = '\0';
		}
		fclose(infile);
	}

	Pose::Pose(char inputline[1280]){
		char posename[128]; char intval[8];
		int temp, count = 0, ind = 0;
		for (int i = 0; i < strlen(inputline) && inputline[count] != ':'; i++) count++; count++;
		for (int i = 0; i < strlen(inputline) && inputline[count] != ':'; i++){
			posename[i] = inputline[count++]; ind++;
		} posename[ind] = '\0'; count++;
		strcpy(name, posename); cout<<"POSE NAME: "<<name<<endl;
		//cout<<inputline<<endl;
		for (int i = 0; i < MAXSERVOS; i++){
			ind = 0;
			for (int j = count; inputline[j] != ',' && inputline[j] != '\0'; j++){
				intval[ind++] = inputline[j];
				count++;
			} count++;
			intval[ind] = '\0';
			temp = atoi(intval);
			servo_val[i] = temp;
			//cout<<"Value #"<<i<<": "<<temp<<", count="<<count<<endl;
		}
	}

	Pose Pose::copy(){
		Pose *retval = new Pose();
		for (int i = 0; i < MAXSERVOS; i++)
			retval->setservo(i, servo_val[i]);
		return *retval;
	}

	void Pose::print(){
		cout<<name<<endl;
		for (int i = 0; i < MAXSERVOS; i++)
			cout<<"#"<<i+1<<": "<<servo_val[i]<<endl;
	}

	Posebank::Posebank(char filename[1280]){
		int s;
		cout<<"Filename="<<filename<<endl;
		l = new Loader(filename); s = l->getsize();
		poses = new Pose *[s];
		cout<<"Number of poses found = "<<s<<endl;
		for (int i = 0; i < s; i++){
			poses[i] = new Pose(l->getinputline(i));
		}
	}
