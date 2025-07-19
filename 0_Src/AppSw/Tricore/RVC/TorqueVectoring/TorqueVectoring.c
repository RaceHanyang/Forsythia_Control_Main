/*
 * TorqueVectoring.c
 * Created on 2019. 12. 10.
 * Author: Dua
 */


/* Includes */
#include "TorqueVectoring.h"

/* Macros */
#define TV_ON	TRUE

#define FD 0.4f

#define F_RATIO 0.4f		// 	R_RATIO = 1 - F_RATIO
#define LR_DEFAULT_RATIO 0.5f
#define MAX_FL_RATIO 0.7f	// 	MIN_FR_RATIO = 1 - MAX_FL_RATIO
#define MAX_FR_RATIO 0.7f	// 	MIN_FL_RATIO = 1 - MAX_FR_RATIO
#define MAX_RL_RATIO 0.5f	//	MIN_RR_RATIO = 1 - MAX_RL_RATIO
#define MAX_RR_RATIO 0.5f	//	MIN_RL_RATIO = 1 - MAX_RR_RATIO

#define MAX_STA 90.0f
#define MIN_STA 10.0f

/* Global Variables */
IFX_EXTERN RVC_t RVC;

/* Function Implementation */
void RVC_TorqueVectoring_run_modeOpen(void)
{
	if (TV_ON)
	{
		float32 deg = fabs(RVC.tv.sta);
		if (deg < MIN_STA)
		{
			deg = MIN_STA;
		}
		if (deg > MAX_STA)
		{
			deg = MAX_STA;
		}
		float32 ratio = (deg - MIN_STA) / (MAX_STA - MIN_STA);
		float32 fl_ratio = LR_DEFAULT_RATIO;
		float32 fr_ratio = LR_DEFAULT_RATIO;
		float32 rl_ratio = LR_DEFAULT_RATIO;
		float32 rr_ratio = LR_DEFAULT_RATIO;
		if (RVC.tv.sta > 0)
		{
			fr_ratio = (ratio * (MAX_FR_RATIO - LR_DEFAULT_RATIO)) + LR_DEFAULT_RATIO;
			fl_ratio = 1.0f - fr_ratio;
			rr_ratio = (ratio * (MAX_RR_RATIO - LR_DEFAULT_RATIO)) + LR_DEFAULT_RATIO;
			rl_ratio = 1.0f - rr_ratio;
		}
		else
		{
			fl_ratio = (ratio * (MAX_FL_RATIO - LR_DEFAULT_RATIO)) + LR_DEFAULT_RATIO;
			fr_ratio = 1.0f - fl_ratio;
			rl_ratio = (ratio * (MAX_RL_RATIO - LR_DEFAULT_RATIO)) + LR_DEFAULT_RATIO;
			rr_ratio = 1.0f - rl_ratio;
		}
		fl_ratio *= F_RATIO;
		fr_ratio *= F_RATIO;
		rl_ratio *= (1.0f - F_RATIO);
		rr_ratio *= (1.0f - F_RATIO);

		float32 max_f_ratio = ((fl_ratio > fr_ratio) ? fl_ratio : fr_ratio);
		float32 max_r_ratio = ((rl_ratio > rr_ratio) ? rl_ratio : rr_ratio);
		float32 max_ratio = ((max_f_ratio > max_r_ratio) ? max_f_ratio : max_r_ratio);

		RVC.torque.frontLeft = RVC.torque.controlled * (fl_ratio / max_ratio);
		RVC.torque.frontRight = RVC.torque.controlled * (fr_ratio / max_ratio);
		RVC.torque.rearLeft = RVC.torque.controlled * (rl_ratio / max_ratio);
		RVC.torque.rearRight = RVC.torque.controlled * (rr_ratio / max_ratio);
	}
	else
	{
		RVC.torque.frontLeft = FD * RVC.torque.controlled;
		RVC.torque.frontRight = FD * RVC.torque.controlled;
		RVC.torque.rearLeft = RVC.torque.controlled;
		RVC.torque.rearRight = RVC.torque.controlled;
	}

	if (RVC.torque.frontLeft < 0)	RVC.torque.frontLeft = 0;
	if (RVC.torque.frontRight < 0)	RVC.torque.frontRight = 0;
	if (RVC.torque.rearLeft < 0)	RVC.torque.rearLeft = 0;
	if (RVC.torque.rearRight < 0)	RVC.torque.rearRight = 0;

	if (RVC.torque.frontLeft > 100)	RVC.torque.frontLeft = 100;
	if (RVC.torque.frontRight > 100)	RVC.torque.frontRight = 100;
	if (RVC.torque.rearLeft > 100)	RVC.torque.rearLeft = 100;
	if (RVC.torque.rearRight > 100)	RVC.torque.rearRight = 100;
}

void RVC_TorqueVectoring_run_mode1(void)
{
	// TODO: TV algorithm
	/*Default*/
	RVC_TorqueVectoring_run_modeOpen();
}
