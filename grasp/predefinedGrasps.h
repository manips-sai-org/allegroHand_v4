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

#endif
