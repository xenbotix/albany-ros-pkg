#ifndef __poses_included__
#define __poses_included__

#include <string.h>

using namespace std;
#define MAXSERVOS 21
#define MAXPOSES 96

 class Loader{
	private:
		int size;
		char linein[MAXPOSES][128];
	public:
		Loader(char filename[128]);
		int getsize(){return size;}
		char *getinputline(int place){return linein[place];}
		
 };

 class Pose{
	private:
		char name[256];
		int servo_val[MAXSERVOS];
	public:
		Pose(){for (int i = 0; i < 32; i++) servo_val[i] = -1;}
		Pose(char inputline[128]);
		Pose copy();
		void setname(char param[256]){strcpy(name, param);}
		void setservo(int place, int val){servo_val[place] = val;}
		int getservo(int place) {return servo_val[place];}
		char *getname(){return name;}
		void print();
 };

 class Posebank{
	private:
		Loader *l;
		Pose **poses;
	public:
		Posebank(char filename[1280]);
		Pose *getpose(int param){return poses[param];}
 };

#endif
