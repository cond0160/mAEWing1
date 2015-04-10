
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"


//#include "aircraft/fenerir_config.h"  // for SIL sim only
#include AIRCRAFT_UP1DIR

extern void get_system_id( double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){

	double t0;
	
	if (time >= 10.0)
	{
		t0 = 10*floor(time/10.0);
	}
	else
	{
		t0 = 0;
	}
	
	// 3-2-1-1 symmetric excitations at 2 Hz to elevators
	controlData_ptr->pitch_cmd_excite = doublet3211(0+t0, time, 0.175, 7*D2R); 		// total duration is 7x duration input
	controlData_ptr->de = controlData_ptr->de + controlData_ptr->pitch_cmd_excite;
	
	// 3-2-1-1 symmetric excitations at 2 Hz to outboard flaps
	controlData_ptr->pitch_cmd_excite = doublet3211(3.5+t0, time, 0.175, 7*D2R); 	// total duration is 7x duration input
	controlData_ptr->l4 = controlData_ptr->l4 + controlData_ptr->pitch_cmd_excite;		
	controlData_ptr->r4 = controlData_ptr->r4 + controlData_ptr->pitch_cmd_excite;		
	
	// 3-2-1-1 symmetric excitations at 2 Hz to inboard flaps
	controlData_ptr->pitch_cmd_excite = doublet3211(7+t0, time, 0.175, 7*D2R);		// total duration is 7x duration input
	controlData_ptr->l1     = controlData_ptr->l1 +  controlData_ptr->pitch_cmd_excite;
	controlData_ptr->r1   = controlData_ptr->r1 +  controlData_ptr->pitch_cmd_excite;
}

