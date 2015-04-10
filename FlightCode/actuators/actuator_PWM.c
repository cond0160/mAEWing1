/*! \file actuator_PWM.c
 *	\brief Functions for PWM set_actuators
 *
 *	\details This file contains functions for driving PWM actuators (servos) via the MPC5200B general purpose timers (GPT).
 *	\ingroup actuator_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: actuator_PWM.c 781 2012-03-15 16:09:37Z murch $
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <cyg/posix/pthread.h>
#include <cyg/kernel/kapi.h>
#include <cyg/cpuload/cpuload.h>
#include <cyg/gpt/mpc5xxx_gpt.h>
#include <cyg/io/mpc5xxx_gpio.h>
#include <cyg/io/i2c_mpc5xxx.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>

#include "../globaldefs.h"
#include "../utils/misc.h"
#include "actuator_interface.h"
#include AIRCRAFT_UP1DIR

/// Absolute limits for PWM output.
/// Note that most servos can respond to commands outside this range, but it is unlikely that commands outside this range will be valid.
#define PWMOUT_1MSEC 2720	///< GPT value corresponding to 1 msec PWM
#define PWMOUT_2MSEC 5440	///< GPT value corresponding to 2 msec PWM

/// Arrays of calibration coefficients using macros defined in aircraft/XXX_config.h.
static double dthr_cal[] = PWMOUT_DTHR_CAL;
static double de_cal[]   = PWMOUT_DE_CAL;
static double da_cal[] = PWMOUT_DA_CAL;
static double l1_cal[]   = PWMOUT_L1_CAL;
static double r1_cal[] = PWMOUT_R1_CAL;
static double l4_cal[] = PWMOUT_L4_CAL;
static double r4_cal[] = PWMOUT_R4_CAL;

/// Compute order of polynomial calibration (length of array - 1)
static int dthr_ord = sizeof(dthr_cal)/sizeof(*dthr_cal) - 1;
static int de_ord   = sizeof(de_cal)/sizeof(*de_cal) - 1;
static int da_ord = sizeof(da_cal)/sizeof(*da_cal) - 1;
static int l1_ord   = sizeof(l1_cal)/sizeof(*l1_cal) - 1;
static int r1_ord = sizeof(r1_cal)/sizeof(*r1_cal) - 1;
static int l4_ord = sizeof(l4_cal)/sizeof(*l4_cal) - 1;
static int r4_ord = sizeof(r4_cal)/sizeof(*r4_cal) - 1;

/// commands in counts
static double dthr_cnts = 0;
static double de_cnts = 0;
static double da_cnts = 0;
static double l1_cnts = 0;
static double r1_cnts = 0;
static double l4_cnts = 0;
static double r4_cnts = 0;

extern void init_actuators(){
	int i;
	
	// initialize PWM I/O channels
	GPT_Init();
	
	// PWM Output
	for(i=0;i<8;i++)
		if( GPT_Open_PWM(i) < 0) fprintf(stderr, "\n GPT_Open_PWM() failed Ch. %d",i);	
}

// Return control outputs based on references and feedback signals.
extern void set_actuators(struct control *controlData_ptr) {
	
	// Enforce surface limits and apply calibration
	dthr_cnts = polyval(dthr_cal, saturation(controlData_ptr->dthr,THROTTLE_MIN,THROTTLE_MAX),dthr_ord);
	de_cnts   = polyval(de_cal, saturation(controlData_ptr->de,ELEVATOR_MIN,ELEVATOR_MAX),de_ord);
	da_cnts = polyval(da_cal, saturation(controlData_ptr->da,AILERON_MIN,AILERON_MAX),da_ord);
	l1_cnts   = polyval(l1_cal, saturation(controlData_ptr->l1,L1_MIN,L1_MAX),l1_ord);
	r1_cnts = polyval(r1_cal, saturation(controlData_ptr->r1,R1_MIN,R1_MAX),r1_ord);
	l4_cnts = polyval(l4_cal, saturation(controlData_ptr->l4,L4_MIN,L4_MAX), l4_ord);
	r4_cnts = polyval(r4_cal, saturation(controlData_ptr->r4,R4_MIN,R4_MAX), r4_ord);	
	
	// Enforce absolute PWM limits for servos and write to mpc5200 PWM channels
	GPT_PWM_Write_Width(PWMOUT_DTHR_CH,  (uint16_t) saturation(dthr_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC));// throttle
	GPT_PWM_Write_Width(PWMOUT_DE_CH,  (uint16_t) saturation(de_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // elevator
	GPT_PWM_Write_Width(PWMOUT_DA_CH, (uint16_t) saturation(da_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // left aileron
	GPT_PWM_Write_Width(PWMOUT_L1_CH,    (uint16_t) saturation(l1_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // L1
	GPT_PWM_Write_Width(PWMOUT_R1_CH, (uint16_t) saturation(r1_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // R1
	GPT_PWM_Write_Width(PWMOUT_L4_CH,    (uint16_t) saturation(l4_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // L4
	GPT_PWM_Write_Width(PWMOUT_R4_CH,    (uint16_t) saturation(r4_cnts,PWMOUT_1MSEC,PWMOUT_2MSEC)); // R4
}


extern void close_actuators(){
	int i;
	
	// PWM Output
	for(i=0;i<8;i++)
		GPT_PWM_Exit(i);
}
