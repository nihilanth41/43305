/*
 *  Copyright (C) 2014 Vision-Guided and Intelligent Robotics Lab
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
 * MODIFIED BY ISMA HADJI ON OCTOBER 2014
 * MODIFICATIONS INCLUDE:  addition of function called MOVETO_XYZOAT_WITH_CHECK
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <fstream>
#include "Puma_OP.h"

extern "C" {
#include "serial_talk.h"
}

int hComm;					// the handle which will hold the opened serial port
bool OPEN_COM = FALSE;
string buff_info;
bool READ = FALSE;


void SAVE_POSITION() {
	FILE *fp;
	fp = fopen("POSITION.txt", "w");
	POSITION a;
	JOINTS b;
	
	READ_POSITION(&a, &b);
	fprintf(fp, "XYZOAT = \n\n");
	fprintf(fp, "%g   %g   %g   %g   %g   %g\n\n", a.X,a.Y,a.Z,a.O,a.A,a.T);
	fprintf(fp, "JOINTS = \n\n");
	fprintf(fp, "%g   %g   %g   %g   %g   %g\n", b.j1,b.j2,b.j3,b.j4,b.j5,b.j6); }


int READ_POSITION(POSITION *pos, JOINTS *jots) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	char buffer[200];	
	sprintf(buffer, "where\n\r");
	writeBuffer(buffer);

	readBuffer(&buff_info);

	float p[100];
	string num;
	int i, j;
	int start = -1, end = -1;
	for (i=0, j=0; buff_info[i] != '\0'; i++) {
		if (start != -1) {
			if (((buff_info[i] >= '0') && (buff_info[i] <= '9')) || (buff_info[i] == '.')) {
				end++; }
			else {
				num.assign(buff_info,start, end-start+1);
				p[j] = atof(num.c_str());
				j++; } }
		if (((buff_info[i] >= '0') && (buff_info[i] <= '9')) || (buff_info[i] == '-')) {
			if (start == -1) {
				start  = end = i; } }
		else {
			if (buff_info[i] != '.') {
				start = end = -1; } } }
	
	pos->X = p[j-18];
	pos->Y = p[j-17];
	pos->Z = p[j-16];
	pos->O = p[j-15];
	pos->A = p[j-14];
	pos->T = p[j-13];

	jots->j1 = p[j-6];
	jots->j2 = p[j-5];
	jots->j3 = p[j-4];
	jots->j4 = p[j-3];
	jots->j5 = p[j-2];
	jots->j6 = p[j-1];
    printf("XYZOAT = \n\n");
    printf("%g   %g   %g   %g   %g   %g\n\n", pos->X,pos->Y,pos->Z,pos->O,pos->A,pos->T);
    printf("JOINTS = \n\n");
    printf("%g   %g   %g   %g   %g   %g\n", jots->j1,jots->j2,jots->j3,jots->j4,jots->j5,jots->j6);
	return 0; }


int MOVE_JOINTv(JOINT jo) {
	return MOVE_JOINT(jo.joint, jo.degree, jo.speed); }


int MOVE_JOINT(int jo, float degree, int spd) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	char buffer[200];	
	sprintf(buffer, "do drive %d, %g, %d\n\r", jo, degree,spd);
	writeBuffer(buffer);
	return 0; }


int MOVETO_JOINTSv(JOINTS jo) {
	return	MOVETO_JOINTS(jo.j1, jo.j2, jo.j3, jo.j4, jo.j5, jo.j6); }


int MOVETO_XYZOATv(POSITION po) {
	return MOVETO_XYZOAT(po.X,po.Y,po.Z,po.O,po.A,po.T); }


int MOVETO_JOINTS_WO_CHECKv(JOINTS jo) {
	return MOVETO_JOINTS_WO_CHECK(jo.j1, jo.j2, jo.j3, jo.j4, jo.j5, jo.j6); }


int MOVETO_XYZOAT_WO_CHECKv(POSITION po) {
	return MOVETO_XYZOAT_WO_CHECK(po.X,po.Y,po.Z,po.O,po.A,po.T); }


int MOVETO_JOINTS_WO_CHECK(float j1, float j2, float j3, float j4, float j5, float j6) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	writeBuffer("point #x\n\r");
	char buffer[200];
	sprintf(buffer, "%g, %g, %g, %g, %g, %g\n\r\n\r", j1, j2 ,j3, j4, j5, j6);
	writeBuffer(buffer);
	//CheckError();
	usleep(100000);
	READ = 0;
	writeBuffer("do move #x\n\r");
	//CheckError();
	return 0; }


int MOVETO_XYZOAT_WO_CHECK(float X, float Y, float Z, float O, float A, float T) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	char buffer[200];	
	sprintf(buffer, "do move trans(%g,%g,%g,%g,%g,%g)\n\r",X,Y,Z,O,A,T);

	writeBuffer(buffer);
	//CheckError();
	return 0; }
bool MOVETO_XYZOAT_WITH_CHECK(float X, float Y, float Z, float O, float A, float T) {
	bool finished=false;
	char buff = -1;
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	char buffer[200];	
	sprintf(buffer, "do move trans(%g,%g,%g,%g,%g,%g)\n\r",X,Y,Z,O,A,T);

	writeBuffer(buffer);
	do{
		serial_read(hComm, &buff, 1);
//		printf("%c\n\n\n",buff);
			if(buff=='.'){
				finished=true;
			}
	}while(!finished);

	return finished;
}
bool MOVETO_XYZOAT_WITH_CHECKv(POSITION po) {
	return MOVETO_XYZOAT_WITH_CHECK(po.X,po.Y,po.Z,po.O,po.A,po.T);
}
int MOVETO_JOINTS(float j1, float j2, float j3, float j4, float j5, float j6) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	writeBuffer("point #x\n\r");	
	char buffer[200];	
	sprintf(buffer, "%g, %g, %g, %g, %g, %g\n\r\n\r", j1, j2 ,j3, j4, j5, j6);
	writeBuffer(buffer);
	CheckError();
	//Sleep(100);
	READ = 0;
	writeBuffer("do move #x\n\r");
	
	return CheckError(); }


int MOVETO_XYZOAT(float X, float Y, float Z, float O, float A, float T) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	char buffer[200];	
	sprintf(buffer, "do move trans(%g,%g,%g,%g,%g,%g)\n\r",X,Y,Z,O,A,T);

	writeBuffer(buffer);

	return CheckError(); }


int setSpeed(int sp) {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	char buffer[15];
	sprintf(buffer, "speed %i\n\r",sp );

	writeBuffer(buffer);
	return 1;
//	return CheckError();
}


int doReady() {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	writeBuffer("do ready\n\r");
	return 1;
//	return CheckError();
}


int doNest() {
	READ = 0;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }
	writeBuffer("do nest\n\r");
	return 1;
//	return CheckError();
}


////////////////Serial Operation///////////////////////////////

int CheckError() {
	int len, start = 0, end = -1;
	bool er = 0;
	string error;
	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	if (!READ) {
		READ = 1;
		readBuffer(&buff_info);
		//cout<<buff_info;
		//printf("%s", buff_info.c_str());
	}

	for (len = buff_info.length()-1; (len > 0)&&((start==0)||(end==-1)); len--) {
		if (buff_info[len] == '*') {
			er = -1;
			if (end == -1)
				end = len;
			else
				start = len; } }

	error.assign(buff_info, start, end-start+1);
	cout<<error;
	return er; }


int InitCal() {
	string data;

	if (!OPEN_COM) {
		serial_close(hComm);
		if ((hComm=serial_open(0, 1, 1)) < 0) {
			cout<<"Error: cannot connect to Puma"<<endl;
			return -1; }
		else OPEN_COM = TRUE; }

	int MachineState = 0;	
	
	while(1) {
		switch (MachineState) {
		case 0:
			writeBuffer("cal\n\r");
			usleep(100000);
			switch (ReadState())	{
			case 0:
				MachineState = 1;
				break;
			case 2:
			case 3:
				MachineState = 4;
				break;
			case 4:
				MachineState = 6;
				break;
			case -1:
				printf("Unresolvable statues...\nPlease turn off the controller and reinitilize the program../n");
				return -1;
				break;
			}
			break;
		case 1:
			switch (ReadState()) {
			case 0:				
				printf("Turn on the controller....\n");
				usleep(3000000);
				break;
			case -1:
				printf("Unresolvable statues...\nPlease turn off the controller and reinitilize the program../n");
				return -1;
				break;
			default:
				printf("Initializing....\n");
				MachineState = 7;
				break;
			}
			break;
		case 2:
			writeBuffer("y\n\r");
			switch(ReadState()) {
			case 2:
				MachineState = 3;
				break;
			}
			break;
		case 3:
			writeBuffer("y\n\r");
			usleep(500000);
			switch(ReadState()) {
			case 1:
			case 4:
				printf("\nInitilization Completed....\n");
				MachineState = 0;
				break;
			}
			break;
		case 4:
			writeBuffer("y\n\r");
			usleep(500000);
			switch(ReadState()) {
			case 1:
				MachineState = 6;
				break;
			case 2:
				MachineState = 3;
				break;
			case 4:
				MachineState = 5;
				break;
			}
			break;
		case 5:
			printf("Turn on the arm power....\n");
			usleep(500000);
			MachineState = 0;
			break;
		case 6:
			writeBuffer("do ready\n\r");
			switch(ReadState()){
			case 1:
				return 0;
			case 2:
				MachineState = 4;
				break;
			case 4:
				MachineState = 5;
				break;
			case -1:
				printf("Unresolvable statues...\nPlease turn off the controller and reinitilize the program../n");
				return -1;
				break;
			}
			break;
		case 7:			
			printf(".");
			usleep(100000);
			switch(ReadState())
			{
			case 2:
				MachineState = 2;
				break;
			case -1:
				printf("Unresolvable statues...\nPlease turn off the controller and reinitilize the program../n");
				return -1;
				break;
			}
			break;
		}
	}
	return 0; }

int ReadState() {
	char buff = 0;
	int i = 0, j =0;
	int state = -1;
	usleep(100000);
	do {
		if ( serial_read(hComm, &buff, 1)==0 ) {
			cout<<"Error in readBuffer\n";
			return -1; }		
		if ((buff=='.') && (state==1))    //the state read as ._ means correct
			break;
		if ((buff == '.') && (state==4))  //the state read as *. means error
			break;
		if ((state!= 1) && (state != -1) && (state != 4))
			break;

		switch(buff) {
		case '.'://possible correct
			state = 1;	
			break;
		case '?':	//the state read as 'r u sure?', means decision(Y/N)
			state = 2;
			break;
		case ':':	//the state read as 'wrong input', means wrong input to decision
			state = 3;
			break;
		case '*':	//possible error
		case 'D':	//'Program Hold'
			state = 4;
			break;
		case 0:	//empty input
			if (i==0)
				state = 0;
			break;
		default:
			if (state == 1)
				state = -1;
			break;
		}
		i++;
		j++;
	} while (j<100);
	
	if (j == 100)
		state = -1;		
	return state; }


int readBuffer(string *data) {
	char word[800];
	char buff = -1;
	int i = -1, j =0;
	int state = 0;
	do {
		if ( serial_read(hComm, &buff, 1)==0 ) {
			printf("%c\n\n\n",buff);
//			cout<<"Error in readBuffer \n";
//			return -1;
		}

		if ((buff=='.') && (state==1))
			break;
		if (buff != 10)
			i++;
		if ((word[i]=buff) == '.')
			state = 1;
		else 
			state = 0;
		if (buff == -1)
			j++;
	} while (j<10);
	if (j!= 10) {
		word[i+1] = '\0';
		*data = string(word); }
	else
		*data = "*Empty Buffer...*\n";
	return 0; }


int writeBuffer(string cmd) {
	char word[80];

	for(unsigned int i=0; i<cmd.length(); i++)
		word[i]=cmd[i];
	//cout<<"hComm: "<<hComm<<endl;
	//cout<<endl<<"Size: " << cmd.length() << " String: "<<cmd<<endl;

	return (!serial_write(hComm, &word[0], cmd.length())); }


