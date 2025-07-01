/*
 * TorqueVectoring.c
 * Created on 2019. 12. 10.
 * Author: Dua
 */

/* Includes */
#include "TorqueVectoring.h"

/* Macros */
#define TVOPEN_LSD_ON FALSE
#define TVOPEN_LSD_GAIN 1.0f
#define TV_ON FALSE

#define FD 0.2f

#define L_MAX_STA 90.0f
#define L_MIN_STA 10.0f
#define R_MAX_STA 90.0f
#define R_MIN_STA 10.0f

#define F_R 0.8f
#define R_R 0.8f

/* Global Variables */
IFX_EXTERN RVC_t RVC;

/* Function Implementation */
void RVC_TorqueVectoring_run_modeOpen(void)
{
	if(TV_ON)
	{
		float32 ratio = 0;
		if(L_MIN_STA <= RVC.tv.sta && RVC.tv.sta <= L_MAX_STA)
			ratio = (RVC.tv.sta - L_MIN_STA) / (L_MAX_STA - L_MIN_STA);
		if(-R_MAX_STA <= RVC.tv.sta && RVC.tv.sta <= -R_MIN_STA)
			ratio = (RVC.tv.sta + R_MIN_STA) / (R_MAX_STA - R_MIN_STA);

		RVC.torque.frontLeft = FD * RVC.torque.controlled * (1 - ratio * F_R);
		RVC.torque.frontRight = FD * RVC.torque.controlled * (1 + ratio * F_R);
		RVC.torque.rearLeft = RVC.torque.controlled * (1 - ratio * R_R);
		RVC.torque.rearRight = RVC.torque.controlled * (1 + ratio * R_R);

		if(RVC.torque.frontLeft < 0)
			RVC.torque.frontLeft = 0;
		if(RVC.torque.frontRight < 0)
			RVC.torque.frontRight = 0;
		if(RVC.torque.rearLeft < 0)
			RVC.torque.rearLeft = 0;
		if(RVC.torque.rearRight < 0)
			RVC.torque.rearRight = 0;

		if(RVC.torque.frontLeft > 100)
			RVC.torque.frontLeft = 100;
		if(RVC.torque.frontRight > 100)
			RVC.torque.frontRight = 100;
		if(RVC.torque.rearLeft > 100)
			RVC.torque.rearLeft = 100;
		if(RVC.torque.rearRight > 100)
			RVC.torque.rearRight = 100;
	}
	else
	{
		RVC.torque.frontLeft = FD * RVC.torque.controlled;
		RVC.torque.frontRight = FD * RVC.torque.controlled;
		RVC.torque.rearLeft = RVC.torque.controlled;
		RVC.torque.rearRight = RVC.torque.controlled;
	}
}

void RVC_TorqueVectoring_run_mode1(void)
{
	// TODO: TV algorithm
	/*Default*/
	RVC_TorqueVectoring_run_modeOpen();
}
