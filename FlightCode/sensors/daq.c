/*! \file daq.c
 *	\brief Data acquisition source code
 *
 *	\details This file implements the init_daq() and get_daq() functions for the UAV.
 *	\ingroup daq_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: daq.c 1014 2014-01-15 18:54:42Z brtaylor $
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
#include "../utils/serial_mpc5200.h"
#include "../utils/misc.h"
#include "../utils/scheduling.h"
#include "../extern_vars.h"
#include "AirData/airdata_interface.h"
#include "AirData/airdata_constants.h"
#include "GPS/gps_interface.h"
#include "IMU/imu_interface.h"
#include "ADC/adc_interface.h"
#include "PWM/pwm_interface.h"
#include "GPIO/gpio_interface.h"
#include "../navigation/nav_interface.h"
#include "daq_interface.h"
#include AIRCRAFT_UP1DIR

/// Arrays of calibration coefficients using macros defined in aircraft/XXX_config.h.
static double incp_thr_cal[] 	= THR_INCP_CAL;
static double incp_pitch_cal[] 	= PITCH_INCP_CAL;
static double incp_yaw_cal[] 	= YAW_INCP_CAL;
static double incp_roll_cal[] 	= ROLL_INCP_CAL;

/// Compute order of polynomial calibration (length of array - 1)
static int incp_thr_ord 	= sizeof(incp_thr_cal)/sizeof(*incp_thr_cal) - 1;
static int incp_pitch_ord   = sizeof(incp_pitch_cal)/sizeof(*incp_pitch_cal) - 1;
static int incp_yaw_ord   	= sizeof(incp_yaw_cal)/sizeof(*incp_yaw_cal) - 1;
static int incp_roll_ord 	= sizeof(incp_roll_cal)/sizeof(*incp_roll_cal) - 1;

/// Low Pass Filter for speed and altitude signals initialization
static double u_alt[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_alt[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

static double u_speed[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_speed[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }


double lp_filter(double signal, double *u, double *y)
{
	const int m=1;  //m = order of denominator of low pass filter

	u[m] = signal;

	y[m] = 0.9608*y[m-1] + 0.0392*u[m-1];	// these coefficients come from a discretized low pass filter with a pole at 2 rad/sec

	u[m-1] = u[m];		// initialize past values for next frame
	y[m-1] = y[m];

	return y[m];
}


void init_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr)
{
	// Assign GPS serial port info.
	sensorData_ptr->gpsData_ptr->baudRate = GPS_BAUDRATE;
	sensorData_ptr->gpsData_ptr->portName = GPS_PORT;
	
	/* Initialize sensors */
	init_gps(sensorData_ptr->gpsData_ptr);		/* GPS */
	init_imu();		/* IMU */
	init_airdata();		/* Ps,Pd */			
	init_gpio();	
		
	// initialize as no data, no lock
	sensorData_ptr->gpsData_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_l_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_r_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_ptr->navValid = 1;
	
    // initialize GPS_TOW to 0, to ensure old_GPS_TOW is set to 0 initially when checking if GPS data is new
	sensorData_ptr->gpsData_ptr->GPS_TOW = 0;
}

void get_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){		

	// local pointers to keep things tidy
	struct imu *imuData_ptr = sensorData_ptr->imuData_ptr;
	struct gps *gpsData_ptr = sensorData_ptr->gpsData_ptr;
	struct airdata *adData_ptr = sensorData_ptr->adData_ptr;
	struct inceptor *inceptorData_ptr = sensorData_ptr->inceptorData_ptr;	
	uint16_t pwm_signals[4];

	int adstatus;

	// IMU Sensor
	read_imu(imuData_ptr);

	// **** IMU Equations ****
	// TODO: correct accels for CG offset
	// **** End IMU ****

	//**** Air Data Sensors ****			
	adstatus = read_airdata(adData_ptr);

	// Set the status bits
	adData_ptr->status &=  0xFF0; // reset the lowest 4 bits.
	adData_ptr->status |=  (uint16_t)adstatus; // set new status.
	
	// Compute pressure altitude; bias removal results in AGL altitude
	adData_ptr->h = AIR_DATA_K1*(1-pow((adData_ptr->Ps)/AIR_DATA_P0,AIR_DATA_K2)) - adData_ptr->bias[0]; //[ m ]
	
	// Compute Indicated Airspeed (IAS). This equation accounts for compressibility effects. Sensor bias is removed prior to calculation
	adData_ptr->ias = copysign(AIR_DATA_K3*sqrt(fabs(pow(fabs((adData_ptr->Pd-adData_ptr->bias[1])/AIR_DATA_P0 +1),AIR_DATA_K4)-1)),adData_ptr->Pd); 

    // Filter altitude and airspeed signals
	adData_ptr->h_filt = lp_filter(adData_ptr->h, u_alt, y_alt);  	    // filtered ALTITUDE
	adData_ptr->ias_filt   = lp_filter(adData_ptr->ias, u_speed, y_speed);	// filtered AIRSPEED
	//**** End Air Data ****

	// Read 4 PWM signals from ATMega328
	read_pwm(&pwm_signals[0]);

	// Apply calibration equations
	inceptorData_ptr->throttle = 	polyval(incp_thr_cal, (double)pwm_signals[THR_INCP_CH],incp_thr_ord);
	inceptorData_ptr->pitch = 	polyval(incp_pitch_cal, (double)pwm_signals[PITCH_INCP_CH]/PWMIN_SCALING,incp_pitch_ord);
	inceptorData_ptr->yaw = 	polyval(incp_yaw_cal, (double)pwm_signals[YAW_INCP_CH]/PWMIN_SCALING,incp_yaw_ord);
	inceptorData_ptr->roll = 	polyval(incp_roll_cal, (double)pwm_signals[ROLL_INCP_CH]/PWMIN_SCALING,incp_roll_ord);

	//**** GPS Sensor(s) ****
	if(-1 == read_gps(gpsData_ptr)){
		if(++gpsData_ptr->read_calls > BASE_HZ)
			gpsData_ptr->err_type = got_invalid;
	}else {gpsData_ptr->read_calls=0;	}

	//**** End GPS ****

	// Read GPIOs; data dump, control mode
	read_gpio(missionData_ptr);
	
}

