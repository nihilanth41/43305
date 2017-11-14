/*
 *  Copyright (C) 2011 Vision-Guided and Intelligent Robotics Lab
 *  Written by G. N. DeSouza <DeSouzaG@missouri.edu>
 *  based on original code by G. N. DeSouza and Youyou Wang
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */


// This program will allow for communication with the PUMA robot through the serial port

#ifndef PumaIOtest_h
#define PumaIOtest_h PumaIOtest_h

	#include <iostream>
	#include <string>
	#include <fstream>
	using namespace std;

	struct JOINT
	{
		int joint;
		float degree;
		int speed;
	};
	struct JOINTS
	{
		float j1;
		float j2;
		float j3;
		float j4;
		float j5;
		float j6;
	};

	struct POSITION
	{
		float X;
		float Y;
		float Z;
		float O;
		float A;
		float T;
	};

	int InitCal();
	int readBuffer(string *data);	// returns puma output into string 'data'
	int writeBuffer(string cmd);		// sends string 'cmd' to puma
	int CheckError();
	int ReadState();

	void SAVE_POSITION();
	int READ_POSITION(POSITION *pos, JOINTS *jots);

	int MOVE_JOINT(int jot, float degree, int speed);
	int MOVE_JOINTv(JOINT joint);

	int MOVETO_JOINTS_WO_CHECK(float j1, float j2, float j3, float j4, float j5, float j6);
	int MOVETO_JOINTS_WO_CHECKv(JOINTS joint);
	int MOVETO_XYZOAT_WO_CHECK(float X, float Y, float Z, float O, float A, float T);
	int MOVETO_XYZOAT_WO_CHECKv(POSITION pos);
	bool MOVETO_XYZOAT_WITH_CHECK(float X, float Y, float Z, float O, float A, float T);
	bool MOVETO_XYZOAT_WITH_CHECKv(POSITION po);
	int MOVETO_JOINTS(float j1, float j2, float j3, float j4, float j5, float j6);
	int MOVETO_JOINTSv(JOINTS joint);
	int MOVETO_XYZOAT(float X, float Y, float Z, float O, float A, float T);
	int MOVETO_XYZOATv(POSITION pos);

	int setSpeed(int sp);
	int doReady();
	int doNest();

#endif





	/*
	// unless otherwise noted a return of -1 is an error and a return of 0 is no error

	// Global Variables

	// Internal Functions
	int openSerial(string port);	// opens communication with 'port'
	int closeSerial();	// closes communication with opened port
	int parseSerial(int &a, int &b, int &c, int &d, int &e, int &f);		// this algorithm gets the six joint or xyz values out of serial output
	string moveNotation(int X,int Y, int Z, int O, int A, int T);	// notates values into proper move command syntax and returns as string
	string jointNotation(int J1, int J2, int J3, int J4, int J5, int J6); // notates joint values into proper get joint command syntax
																											// and returns as string
	// User Functions
	int pumaInit(string port);	// initializes puma for operation, must be called before any other functions
	int pumaNest();	// returns puma to nested position, should be called at end of program

	int movePumaXYZ(int &X,int &Y, int &Z, int &O, int &A, int &T);	// moves puma to the position given
	int movePumaJoint(int &J1, int &J2, int &J3, int &J4, int &J5, int &J6);	// moves puma based on the 6 joint values passed
	int readPumaXYZ(int &X,int &Y, int &Z, int &O, int &A, int &T);	// gets puma's current position in the format (x,y,z,o,a,t)
	int readPumaJoint(int &J1, int &J2, int &J3, int &J4, int &J5, int &J6);	// gets puma's current position in joint format
*/
