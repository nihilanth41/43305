/*
 *  Copyright (C) 2014 Vision-Guided and Intelligent Robotics Lab
 *  Written by G. N. DeSouza <DeSouzaG@missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 */

// 
//  Based on original code
//  by G. N. DeSouza 4/10/2000
//  and Youngrock Yoon <yoony@purdue.edu>
//
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "serial_talk.h"


//void signal_handler_IO (int status);   /* definition of signal handler */

int serial_read(int port_id, char *x, int num_bytes) {
	return read(port_id, x, num_bytes); }


int serial_write(int port_id, char *x, int num_bytes) {
	return write(port_id, x, num_bytes); }


void set_baud(int port, int final_baud){
    int port_id;
    struct termios t_port;
    #ifdef USB
	   char COM_PORT[16]="/dev/ttyUSB0";
    #else
	   char COM_PORT[16]="/dev/ttyS0";
    #endif
    int baud_rate[] = {B4800,B9600,B19200,B38400,B57600,B115200};

    #ifdef USB
	   printf("USB case");
	   COM_PORT[11] = COM_PORT[11] + port;
    #else
	   printf("Serial case");
	   COM_PORT[9] = COM_PORT[9] + port;
    #endif
    printf("Initializing communications port %s...", COM_PORT);

    /* open the device */
    if ((port_id = open(COM_PORT, O_RDWR)) == -1){
        printf("can not open device %s", COM_PORT);
        exit(1); }

    if (tcgetattr(port_id, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
        exit(1); }

    t_port.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); // input mode
        /* no SIGINT on BREAK, CRNL off, parity off,8th bit strip off, output flow ctrl off */
    t_port.c_oflag &= ~(OPOST); // output mode
        /* output processing off */
    t_port.c_cflag &= ~(CSIZE | PARENB); // control mode
        /* clear size bits, parity off */
    t_port.c_cflag |=  baud_rate[final_baud] | CS8 | CREAD | CLOCAL ;
    t_port.c_cflag &= ~(CRTSCTS | CSTOPB);
        // 4800 9600 19200 38400 57600 115200
        /* 8 bits/char, enable receiver, close modem when exit */
    t_port.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        /* echo off, canonical mode off,extended input processing off, signal chars off */

    t_port.c_cc[VMIN] = 0;//MIN_MSG_SIZE;       // wait for the amount of VMIN byte
    t_port.c_cc[VTIME] = 10;    /* waits 1/10ths of sec for reply */

    if (cfsetspeed(&t_port, baud_rate[final_baud]) < 0) {
		printf("error initiallizing %s", COM_PORT);
		exit(1); }
    if (tcsetattr(port_id, TCSANOW, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
        exit(1); }
    if (tcgetattr(port_id, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
        exit(1); } }


int serial_open(int port, int initial_baud, int final_baud){
	int port_id;
	struct termios t_port;
    #ifdef USB
		char COM_PORT[16]="/dev/ttyUSB0";
    #else
	   char COM_PORT[16]="/dev/ttyS0";
    #endif
	int baud_rate[] = {B4800,B9600,B19200,B38400,B57600,B115200};

    #ifdef USB
	   printf("USB case");
	   COM_PORT[11] = COM_PORT[11] + port;
    #else
	   printf("Serial case");
	   COM_PORT[9] = COM_PORT[9] + port;
    #endif
	printf("Initializing communications port %s...", COM_PORT);

	/* open the device */
	if ((port_id = open(COM_PORT, O_RDWR)) == -1){
		printf("can not open device %s", COM_PORT);
		exit(1); }

	if (tcgetattr(port_id, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
		exit(1); }

	t_port.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); // input mode
		/* no SIGINT on BREAK, CRNL off, parity off,8th bit strip off, output flow ctrl off */
	t_port.c_oflag &= ~(OPOST); // output mode
		/* output processing off */
	t_port.c_cflag &= ~(CSIZE | PARENB); // control mode
		/* clear size bits, parity off */
	t_port.c_cflag |=  baud_rate[final_baud] | CS8 | CREAD | CLOCAL ;
	t_port.c_cflag &= ~(CRTSCTS | CSTOPB);
		// 4800 9600 19200 38400 57600 115200
		/* 8 bits/char, enable receiver, close modem when exit */
	t_port.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
		/* echo off, canonical mode off,extended input processing off, signal chars off */

	t_port.c_cc[VMIN] = 0;//MIN_MSG_SIZE;		// wait for the amount of VMIN byte
	t_port.c_cc[VTIME] = 10;    /* waits 1/10ths of sec for reply */

	if (cfsetspeed(&t_port, baud_rate[final_baud]) < 0) {
		printf("error initiallizing %s", COM_PORT);
		exit(1); }
	if (tcsetattr(port_id, TCSANOW, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
		exit(1); }
    //sleep(2);
	if (tcgetattr(port_id, &t_port) < 0){
		printf("error initiallizing %s", COM_PORT);
		exit(1); }

	return port_id; }


void serial_close(int ofp){
	close(ofp); }


