/*
 * TorqueVectoring.c
 * Created on 2019. 12. 10.
 * Author: Dua
 */

/* Includes */
#include "TorqueVectoring.h"
#include <math.h>

/* Macros */
#define TVOPEN_LSD_ON FALSE
#define TVOPEN_LSD_GAIN 1.0f

#define FD 0.2f

/* Global Variables */
IFX_EXTERN RVC_t RVC;

TV_t tv = {
	.enable.v_x = 0.0f,			// m/s
    .enable.delta_sta = 0.07f,	// rad
    .enable.k_1 = 0.25f,		

    .error.k_2 = 0.0f,
    .error.l = 0.8f,

    .pid.k_p = 0.0f,
    .pid.k_i = 0.0f,
	.pid.i_err = 0.0f;
    .pid.k_d = 0.0f,
	.pid.pre_err = 0.0f;
	.pid.dt = 0.001f;	// s
    .pid.k_b = 0.0f,

    .limit.max_delta_trq = 1.0f,

    .tv.t = 1.0f
};

double Enable(double v_x, double delta_sta);
double Error(double v_x, double delta_sta, double lamda);
double pid(double enable, double error, double limit);

/* Function Implementation */
void RVC_TorqueVectoring_run_modeOpen(void)
{
	// if(TVOPEN_LSD_ON == TRUE)
	// {
	// 	// RVC.slip.axle

	// }
	// else
	// {
	// 	float32 fd = 2*RVC.torque.frontDist;
	// 	RVC.torque.frontLeft = fd * RVC.torque.controlled;
	// 	RVC.torque.frontRight = fd * RVC.torque.controlled;
	// 	RVC.torque.rearLeft = RVC.torque.controlled;
	// 	RVC.torque.rearRight = RVC.torque.controlled;
	// }

	//	float32 fd = 2 * RVC.torque.frontDist;
	//	RVC.torque.frontLeft = fd * RVC.torque.controlled;
	//	RVC.torque.frontRight = fd * RVC.torque.controlled;
	RVC.torque.frontLeft = FD * RVC.torque.controlled;
	RVC.torque.frontRight = FD * RVC.torque.controlled;
	RVC.torque.rearLeft = RVC.torque.controlled;
	RVC.torque.rearRight = RVC.torque.controlled;
}

void RVC_TorqueVectoring_run_mode1(void)
{
	// TODO: TV algorithm
	/*Default*/
	RVC_TorqueVectoring_run_modeOpen();
}

double Enable(double v_x, double delta_sta)
{
	double enable_v_x = (v_x < tv.enable.v_x) ? 0.0f : 1.0f;
	double enable_delta_sta = ((fabs(delta_sta) - tv.enable.delta_sta) * tv.enable.k_1 / tv.enable.delta_sta);
	if (enable_delta_sta > 1)	enable_delta_sta = 1;
	if (enable_delta_sta < 0)	enable_delta_sta = 0;
	return (enable_v_x * enable_delta_sta);
}

double Error(double v_x, double delta_sta, double lamda, double enable)
{
	if (enable == 0)	return 0;
	return (lamda / v_x) - (delta_sta * tv.error.k_2);
}

double pid(double enable, double error, double limit, double bk)
{
	double p_err = tv.pid.k_p * error;
	tv.pid.i_err += (tv.pid.k_i * error - bk) * tv.pid.dt;
	
	tv.pid.pre_err = error;
}
