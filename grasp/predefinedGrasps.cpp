#include "predefinedGrasps.h"
#include "rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>

extern double q_des[MAX_DOF];

void SetGraspAndGains(std::string grasp_name, double(&kp)[16], double(&kv)[16])
{
	printf{"hola"}
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
		throw exception;
	}
}


