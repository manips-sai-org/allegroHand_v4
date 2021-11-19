
#include "rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>

// ROCK-SCISSORS-PAPER(LEFT HAND)
static double pos1[] = {
	-0.1194, 1.2068, 1.0, 1.4042,
	-0.0093, 1.2481, 1.4073, 0.8163,
	0.1116, 1.2712, 1.3881, 1.0122,
	0.6017, 0.2976, 0.9034, 0.7929};

static double kp1[] = {
	500, 800, 900, 500,
	500, 800, 900, 500,
	500, 800, 900, 500,
	1000, 700, 600, 600
};
static double kd1[] = {
	25, 50, 55, 40,
	25, 50, 55, 40,
	25, 50, 55, 40,
	50, 50, 50, 40
};