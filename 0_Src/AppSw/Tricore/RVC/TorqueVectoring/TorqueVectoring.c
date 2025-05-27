/*
 * TorqueVectoring.c
 * Created on 2019. 12. 10.
 * Author: Dua
 */


/* Includes */
#include "TorqueVectoring.h"

/* Macros */
#define TVOPEN_LSD_ON		FALSE
#define TVOPEN_LSD_GAIN		1.0f

#define FD 0.2f

/* Global Variables */
IFX_EXTERN RVC_t RVC;

TV_t tv =
{
	.enable.v_x = 0.0f,
	.enable.delta_sta = 0.0f,
	.enable.k_1 = 0.25f,

	.error.k_2 = 0.0f,
	.error.i = 0.0f,

	.pid.k_p = 0.0f,
	.pid.k_i = 0.0f,
	.pid.k_d = 0.0f,
	.pid.k_b = 0.0f,

	.limit.max_delta_trq = 1.0f,

	.tv.t = 1.0f
};

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
