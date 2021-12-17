#include "rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>
#include <stdexcept>

extern double q_des[MAX_DOF];
// ----------------------
//    Predefined grasps
// ----------------------
double home[] = {
	0.094000,0.264248,0.001331,0.905473,
	0.062134,0.211345,0.122759,1.074300,
	0.069945,0.417897,0.051660,1.041635,
	1.280231,0.145660,0.261052,0.395795};

double ready[] = {
	0.044559,0.657735,0.275965,0.908136,
	0.115747,0.763629,0.135364,0.903431,
	0.060359,0.731675,0.105539,1.031250,
	1.200877,0.334371,0.538792,0.038967};

// -----------
//    Gains
// -----------
double kp_predef[] = {
    5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0,
    2.5, 2.5, 2.5, 2.5 //thumb
};
double kv_predef[] = {
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09
};


void SetGraspAndGains(std::string grasp_name, double(&kp)[16], double(&kv)[16])
{
	if (grasp_name == "home"){
		for (int i=0; i<16; i++){ 
			q_des[i] = home[i];
			kp[i] = kp_predef[i];
            kv[i] = kv_predef[i];
        }
	}
    else if(grasp_name == "ready"){
    	for (int i=0; i<16; i++){
			q_des[i] = ready[i];
			kp[i] = kp_predef[i];
            kv[i] = kv_predef[i];
        }
    }
	else{
		throw std::invalid_argument( "received invalid arg" );
	}
}