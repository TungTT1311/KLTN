#include "robot_move.h"

// Pin Definitions
// Pin Definitions
const int right_output_pwm_pin1 = 33; //pwm
const int right_output_pin2 = 31; //


const int left_output_pin_1 = 29; // 
const int left_output_pwm_pin_2 = 32; // pwm


bool end_this_program = false;


inline void delay(int s){
	this_thread::sleep_for(chrono::seconds(s));
}

void signalHandler (int s){
	end_this_program = true;
}

void Distance(int knowWidth, int frame_width)
{
 // int focalLength = (864/2)*sqrt(3);
    int focalLength = (frame_width/2)*sqrt(3);
  // int realheight = 1750;
  // int imageheight = 720;

  // int objectheight = rec.height;
  // int sensorheight = 10;
  
  // cout << "Distance = " << (focalLength*realheight*imageheight)/(objectheight*sensorheight) << endl;
  
  cout << "Distance = " << 60*focalLength/knowWidth << endl;
  cout << endl;
}

