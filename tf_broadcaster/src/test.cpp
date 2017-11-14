//============================================================================
// Name        : Lab5.cpp
// Author      : isma Hadji
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/*
 *  Copyright (C) 2011 Vision-Guided and Intelligent Robotics Lab
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
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */

#include <stdlib.h>
#include "Puma_OP.h"

int main(int argc, char **argv) {

	
	if(argc != 7)
	{
		printf("Error Arguments...\n");
		return -1;
	}
	POSITION p;
	p.X = atof(argv[1]);
	p.Y = atof(argv[2]);
	p.Z = atof(argv[3]);
	p.O = atof(argv[4]);
	p.A = atof(argv[5]);
	p.T = atof(argv[6]);

	MOVETO_XYZOAT_WO_CHECKv(p);
	return 0;
} // int main

