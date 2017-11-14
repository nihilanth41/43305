/*
 *  Copyright (C) 2000 RVL - Robot Vision Lab (http://RVL.www.ecn.purdue.edu)
 *  Written by Guilherme Nelson DeSouza <gnelson@purdue.edu>
 *          and Youngrock Yoon <yoony@purdue.edu>
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
//  by G. N. DeSouza 4/10/2000
//
//
#define MAX_MSG_SIZE	80
#define MIN_MSG_SIZE	1

#define USB 1

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

//void signal_handler_IO (int status);   /* definition of signal handler */

int serial_read(int port_id, char *x, int num_bytes);

int serial_write(int port_id, char *x, int num_bytes);

void set_baud(int port, int final_baud);

int serial_open(int port, int initial_baud, int final_baud);

void serial_close(int port_id);


