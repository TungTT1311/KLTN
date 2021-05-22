#include "robot_move.h"

// Pin Definitions
int output_pin1 = 29; // BOARD pin 12, BCM pin 18
int output_pin2 = 31; // BOARD pin 12, BCM pin 18
bool end_this_program = false;


inline void delay(int s){
	this_thread::sleep_for(chrono::seconds(s));
}

void signalHandler (int s){
	end_this_program = true;
}

// 
void Distance(int knowWidth)
{
  int focalLength = (864/2)*sqrt(3);
  // int realheight = 1750;
  // int imageheight = 720;

  // int objectheight = rec.height;
  // int sensorheight = 10;
  
  // cout << "Distance = " << (focalLength*realheight*imageheight)/(objectheight*sensorheight) << endl;
  
  cout << "Distance = " << 60*focalLength/knowWidth << endl;
  cout << endl;
}
