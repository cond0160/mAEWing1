
#ifndef FENRIR_DATALOG_H_
#define FENRIR_DATALOG_H_
	
// Datalogging setup
#define LOG_ARRAY_SIZE 90000 ///< Number of data points in the logging array. 50 Hz * 60 sec/min * 30 minutes = 90000

#define NUM_DOUBLE_VARS 6	///< Number of variables that will be logged as doubles
#define NUM_FLOAT_VARS 53	///< Number of variables that will be logged as floats
#define NUM_INT_VARS 2		///< Number of variables that will be logged as ints
#define NUM_SHORT_VARS 7	///< Number of variables that will be logged as shorts

//Names of matlab variables MUST match pointers!!

/// char array of variable names for doubles
char* saveAsDoubleNames[NUM_DOUBLE_VARS] = {
		"lat", "lon","alt",
		"navlat", "navlon", "navalt"};

/// double pointer array to variables that will be saved as doubles
double* saveAsDoublePointers[NUM_DOUBLE_VARS] = {
		&gpsData.lat, &gpsData.lon, &gpsData.alt,
		&navData.lat, &navData.lon, &navData.alt};

/// char array of variable names for floats
char* saveAsFloatNames[NUM_FLOAT_VARS] = {
 			"time",
			"p", "q", "r", 
			"ax", "ay", "az", 
			"hx", "hy", "hz",
			"h", "ias", 
			"h_filt","ias_filt",
			"Ps","Pd", 
			"vn", "ve", "vd",
			"GPS_TOW",
			"navvn", "navve","navvd",
			"phi", "theta", "psi",
			"p_bias", "q_bias", "r_bias",
			"ax_bias", "ay_bias", "az_bias",
			"thr_incp","pitch_incp", "yaw_incp",
			"roll_incp",
			"de", "da", "l1", "r1",
			"l4", "r4", "dthr",
			"pitch_cmd_pilot","pitch_cmd_damper","pitch_cmd_excite",
			"roll_cmd_pilot","roll_cmd_damper","roll_cmd_excite"};
								
/// double pointer array to variables that will be saved as floats
double* saveAsFloatPointers[NUM_FLOAT_VARS] = {
			&imuData.time,
			&imuData.p, &imuData.q, &imuData.r,
			&imuData.ax, &imuData.ay, &imuData.az,
			&imuData.hx, &imuData.hy, &imuData.hz,
			&adData.h, &adData.ias, 
			&adData.h_filt, &adData.ias_filt,
			&adData.Ps, &adData.Pd, 
			&gpsData.vn, &gpsData.ve, &gpsData.vd, 
			&gpsData.GPS_TOW, 
			&navData.vn, &navData.ve, &navData.vd,
			&navData.phi, &navData.the, &navData.psi,
			&navData.gb[0], &navData.gb[1], &navData.gb[2],
			&navData.ab[0], &navData.ab[1], &navData.ab[2],
			&inceptorData.throttle, &inceptorData.pitch, &inceptorData.yaw,
			&inceptorData.roll,
			&controlData.de, &controlData.da, &controlData.l1, &controlData.r1, 
			&controlData.l4, &controlData.r4, &controlData.dthr,
			&controlData.pitch_cmd_pilot, &controlData.pitch_cmd_damper, &controlData.pitch_cmd_excite,
			&controlData.roll_cmd_pilot, &controlData.roll_cmd_damper, &controlData.roll_cmd_excite};

/// char array of variable names for ints
char* saveAsIntNames[NUM_INT_VARS] = {"imuStatus","gpsStatus"};

/// int32_t pointer array to variables that will be saved as ints
int32_t* saveAsIntPointers[NUM_INT_VARS] = {(int32_t *)&imuData.err_type,(int32_t *)&gpsData.err_type};


/// char array of variable names for shorts
char* saveAsShortNames[NUM_SHORT_VARS] = {"mode", "satVisible", "navValid","cpuLoad","adStatus","run_num","run_excitation"};

/// uint16_t pointer array to variables that will be saved as shorts
uint16_t* saveAsShortPointers[NUM_SHORT_VARS] = {&missionData.mode, &gpsData.satVisible,
												  &gpsData.navValid,&cpuLoad,&adData.status,&missionData.run_num,&missionData.run_excitation};
#endif	

