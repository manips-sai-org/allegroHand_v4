#ifndef _PREDEF_GRASP_H
#define _PREDEF_GRASP_H

#include <string>

void SetGraspAndGains(std::string grasp_name, double(&kp)[16], double(&kv)[16]);
// *
//  * @brief      sets the joint angles corresponding to a predefined grasp
//  * 			   also sets the gains for that grasp. You can set new grasps 
//  * 	           and gains for those grasps below :) 
//  * @param      predefined grasp name from the list below
//  * @throw      an error message and default gains if the specified grasp
//  *             name does not exist

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
double kp_default[] = {
    5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0,
    5.0, 5.0, 5.0, 5.0,
    2.5, 2.5, 2.5, 2.5 //thumb
};
double kv_default[] = {
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09,
    0.09, 0.09, 0.09, 0.09
};
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

#endif
