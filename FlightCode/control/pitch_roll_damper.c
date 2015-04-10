/*! \file baseline_control.c
 *	\brief baseline controller source code
 *
 *	\details The baseline controller is a classical PI design that tracks pitch angle (theta) and roll angle (phi).
 *	A yaw damper and roll damper is used for the lateral axis, and a pitch damper is used for the longitudinal axis.
 *
 *	\ingroup control_fcns
 *
 *	\author University of Minnesota
 *	\author Aerospace Engineering and Mechanics
 *	\copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: baseline_control.c 929 2012-10-29 16:50:59Z joh07594 $
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"



// ***********************************************************************************

//#include "../aircraft/fenrir_config.h"  // for SIL_Sim
#include AIRCRAFT_UP1DIR            // for Flight Code

// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double roll_damper (double rollrate);
static double pitch_damper(double pitchrate);
static double elev_input(double de_in);
static double ail_input(double da_in);

#ifdef AIRCRAFT_FENRIR
	static double roll_gain  = 0;  // gain for roll damper
	static double pitch_gain = -0.7;  // gain for pitch damper
#endif

/// *****************************************************************************************

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {
/// Return control outputs based on references and feedback signals.

	double p     	  = sensorData_ptr->imuData_ptr->p; // Roll rate
	double q     	  = sensorData_ptr->imuData_ptr->q; // Pitch rate
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	
	controlData_ptr->pitch_cmd_pilot = elev_input(pitch_incp);
	controlData_ptr->pitch_cmd_damper = pitch_damper(q);
	controlData_ptr->roll_cmd_pilot = ail_input(roll_incp);
	controlData_ptr->roll_cmd_damper = roll_damper(p);
	
	controlData_ptr->dthr = 0; 		// throttle [ND]
    controlData_ptr->de = controlData_ptr->pitch_cmd_pilot + controlData_ptr->pitch_cmd_damper;		// Elevator deflection [rad]
	controlData_ptr->da = controlData_ptr->roll_cmd_pilot + controlData_ptr->roll_cmd_damper; 		// Aileron deflection [rad]
	controlData_ptr->l1 = 0;		// L1 [rad]
    controlData_ptr->r1 = 0; 		// R1 [rad]
	controlData_ptr->l4 = controlData_ptr->pitch_cmd_pilot + controlData_ptr->roll_cmd_pilot; 		// L4 [rad]
	controlData_ptr->r4 = controlData_ptr->pitch_cmd_pilot - controlData_ptr->roll_cmd_pilot; 		// R4 [rad]
}

static double roll_damper (double rollrate)
{
	double da;
	    // roll damper
	da  =  -1 * roll_gain*rollrate;
	
    return da;
}

static double pitch_damper(double pitchrate)
{
	double de;
       // pitch damper
    de =  -1 * pitch_gain*pitchrate;    // Elevator output

	return de;  //rad
}

static double elev_input(double de_in)
{
	double de;
    de =  ((de_in-.03785)*39.498370692209)*D2R; // pilot input stick shaping    

	return de;  //rad
}

static double ail_input(double da_in)
{
	double da;
    da =  -1*((da_in-.0158)*39.498370692209)*D2R; // pilot input stick shaping   

	return da;  //rad
}

// Reset parameters to initial values
extern void reset_control(struct control *controlData_ptr){

	controlData_ptr->dthr = 0; // throttle
	controlData_ptr->de   = 0; // elevator
	controlData_ptr->da   = 0; // aileron
	controlData_ptr->l1   = 0; // l1
	controlData_ptr->r1 = 0; // r1
	controlData_ptr->l4 = 0; // l4
	controlData_ptr->r4 = 0; // r4

}

