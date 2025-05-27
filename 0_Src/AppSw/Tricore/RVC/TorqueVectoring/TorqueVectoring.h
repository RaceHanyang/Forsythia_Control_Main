/*
 * TorqueVectoring.h
 * Created on 2019. 12. 10.
 * Author: Dua
 */

#ifndef TORQUEVECTORING_H_
#define TORQUEVECTORING_H_

/* Includes */
#include "RVC.h"
#include "RVC_privateDataStructure.h"

/*************************** Data Structures *********************************/
typedef struct
{
	struct
	{
		double v_x;       // enable limit of velocity
		double delta_sta; // enable limit of steering angle
		double k_1;       // gain for enable limit of steering angle
	} enable;

	struct
	{
		double k_2; // (steering angle) - (wheel gear ratio)
		double i;   // |(front accel) - (center of mass)|
	} error;

	struct
	{
		double k_p; // proportional gain
		double k_i; // integral gain
		double k_d; // derivative gain
		double k_b; // inverse calculation constant == k_i
	} pid;

	struct
	{
		double max_delta_trq; // limit of maximum delta torque
	} limit;

	struct
	{
		double t; // constant of transfer function constant
	} tv;

} TV_t;

/* Functionc Prototypes */
IFX_EXTERN void RVC_TorqueVectoring_run_modeOpen(void);
IFX_EXTERN void RVC_TorqueVectoring_run_mode1(void);
#endif