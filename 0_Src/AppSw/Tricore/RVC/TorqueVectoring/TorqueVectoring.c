/*
 * TorqueVectoring.c
 * Created on 2019. 12. 10.
 * Author: Dua
 */


/* Includes */
#include "TorqueVectoring.h"

/* Macros */
#define FR_ON FALSE

#define X0 0.0f // 0% don't adjust it
#define Y0 0.1f

#define X1 0.2f // 20%
#define Y1 0.18f

#define X2 0.6f // 60%
#define Y2 0.46f

#define X3 1.0f // 100% don't adjust it
#define Y3 0.5f

#define TV_ON   TRUE

#define FD 0.4f

#define F_RATIO 0.4f      //    R_RATIO = 1 - F_RATIO
#define LR_DEFAULT_RATIO 0.5f
#define MAX_FL_RATIO 0.7f   //    MIN_FR_RATIO = 1 - MAX_FL_RATIO
#define MAX_FR_RATIO 0.7f   //    MIN_FL_RATIO = 1 - MAX_FR_RATIO
#define MAX_RL_RATIO 0.5f   //   MIN_RR_RATIO = 1 - MAX_RL_RATIO
#define MAX_RR_RATIO 0.5f   //   MIN_RL_RATIO = 1 - MAX_RR_RATIO

#define MAX_STA 90.0f
#define MIN_STA 10.0f

/* Global Variables */
IFX_EXTERN RVC_t RVC;

/* Function Implementation */
static inline float32 my_absf(float32 x) {
    return (x < 0.0f) ? -x : x;
}

void RVC_TorqueVectoring_run_modeOpen(void)
{
	float32 front_ratio = F_RATIO;
	if (FR_ON)
	{
		float32 u = RVC.torque.desired / 100.0f;

		if (u < 0.0f)
		{
			u = 0.0f;
		}
		if (u > 1.0f)
		{
			u = 1.0f;
		}
      
		if(u <= X1)
		{
			front_ratio =  Y0 + (Y1 - Y0) * (u - X0) / (X1 - X0);
		}
		else if(u <= X2)
		{
			front_ratio =  Y1 + (Y2 - Y1) * (u - X1) / (X2 - X1);
		}
		else if(u <= X3)
		{
			front_ratio =  Y2 + (Y3 - Y2) * (u - X2) / (X3 - X2);
		}
		else
		{
			front_ratio = Y3;
		}
	}

	if (TV_ON)
	{
		float32 deg = my_absf(RVC.tv.sta);
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
		fl_ratio *= front_ratio;
		fr_ratio *= front_ratio;
		rl_ratio *= (1.0f - front_ratio);
		rr_ratio *= (1.0f - front_ratio);

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

	if (RVC.torque.frontLeft < 0)   RVC.torque.frontLeft = 0;
	if (RVC.torque.frontRight < 0)   RVC.torque.frontRight = 0;
	if (RVC.torque.rearLeft < 0)   RVC.torque.rearLeft = 0;
	if (RVC.torque.rearRight < 0)   RVC.torque.rearRight = 0;

	if (RVC.torque.frontLeft > 100)   RVC.torque.frontLeft = 100;
	if (RVC.torque.frontRight > 100)   RVC.torque.frontRight = 100;
	if (RVC.torque.rearLeft > 100)   RVC.torque.rearLeft = 100;
	if (RVC.torque.rearRight > 100)   RVC.torque.rearRight = 100;
}

void RVC_TorqueVectoring_run_mode1(void)
{
	// TODO: TV algorithm
	/*Default*/
	RVC_TorqueVectoring_run_modeOpen();
}
