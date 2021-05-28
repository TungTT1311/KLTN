#ifndef ROBOT_MOVE_H
#define ROBOT_MOVE_H
#pragma once
#include <iostream>
// for delay function.
#include <chrono> 
#include <thread>

// for signal handling
#include <signal.h>
#include <math.h>
#include <../include/JetsonGPIO.h>



using namespace std;
using namespace GPIO;

inline void delay(int s);
void signalHandler (int s);
void Distance(int knowWidth, int frame_width);

//void SetupMotor();

//void MotorLeft(int Direct, int Speed);
//void MotorRight(int Direct, int Speed);

//void ControlMotor(int LeftDirect, int LeftSpeed, int RightDirect, int RightSpeed);
//void Move(int frameWidth, int recX, int recWidth);
#endif
