
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "mission_interface.h"

extern void run_mission(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct mission *missionData_ptr) {
	if(fabs(sensorData_ptr->inceptorData_ptr->yaw) > 0.4){
		missionData_ptr->run_excitation = 1;
	}
	else{
		missionData_ptr->run_excitation = 0;
	}
}
