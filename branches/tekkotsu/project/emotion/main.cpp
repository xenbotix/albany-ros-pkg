#include <iostream>
#include <stdlib.h>
#include "emotion.h"

using namespace std;

int main(int argc, char *argv[]){
	int xx, yy;
	Emotion e;
	Pose *p;
	xx = atoi(argv[1]);
        yy = atoi(argv[2]);  yy = 100 - yy; if (yy < 0) yy = 0;
	cout<<argv[1]<<" and "<<argv[2]<<endl;
/*	cout<<"Current State: "<<e.currentstate()<<endl;
	e.move(0,-30);
	cout<<"Current State: "<<e.currentstate()<<endl;
	e.move(0,-30);
	cout<<"Current State: "<<e.currentstate()<<endl;
	e.move(50,-30);
	cout<<"Current State: "<<e.currentstate()<<endl;
	e.move(-50,+30);
	cout<<"Current State: "<<e.currentstate()<<endl;
	e.move(0, 20);
	cout<<"Current State: "<<e.currentstate()<<endl; */
	cout<<"Current State: "<<e.currentstate()<<endl;
	p = e.currentpose();
	//p->print();
	//e.move(-25, -25);
	e.setpos(xx, yy);
	cout<<"Current State: "<<e.currentstate()<<endl;
	p = e.currentpose();
/*
	cout<<"TESTING "; p->print();
for (int i = 0; i < 101; i++){
	for (int j = 0; j < 100; j++){
		e.move(1,0);
		cout<<"Current State: "<<e.currentstate()<<endl;
	}
	
	e.move(-100, 1);
	cout<<" *NEW LINE* ======================================= \n Current State: "<<e.currentstate()<<endl;

} 
	cout<<"Current State: "<<e.currentstate()<<endl;
*/
	e.writeposefile();
	e.useposefile();
	return 0;
}
