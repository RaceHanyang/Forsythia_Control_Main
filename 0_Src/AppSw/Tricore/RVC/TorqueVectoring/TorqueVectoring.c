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

#define REGEN_ON FALSE

#define FD 0.4f

#define F_RATIO 0.375f      //    R_RATIO = 1 - F_RATIO
#define LR_DEFAULT_RATIO 0.5f
#define MAX_FL_RATIO 0.64f   //    MIN_FR_RATIO = 1 - MAX_FL_RATIO
#define MAX_FR_RATIO 0.64f   //    MIN_FL_RATIO = 1 - MAX_FR_RATIO
#define MAX_RL_RATIO 0.6f   //   MIN_RR_RATIO = 1 - MAX_RL_RATIO
#define MAX_RR_RATIO 0.6f   //   MIN_RL_RATIO = 1 - MAX_RR_RATIO

#define MAX_STA 90.0f
#define MIN_STA 10.0f

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Global Variables */
IFX_EXTERN RVC_t RVC;

/* Function Implementation */
static inline float32 my_absf(float32 x) {
    return (x < 0.0f) ? -x : x;
}

static inline float32 clamp(float32 val, float32 min, float32 max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

float32 lut1d(float32 x, const float32* x_data, const float32* y_data, int length)
{
	if (x <= x_data[0])
	{
		return y_data[0];
	}
	else if (x >= x_data[length - 1])
	{
		return y_data[length - 1];
	}
	else
	{
		for (int i = 0; i < length - 1; i++)
		{
			if (x >= x_data[i] && x < x_data[i + 1])
			{
				float32 t = (x - x_data[i]) / (x_data[i + 1] + x_data[i]);
				return y_data[i] + t * (y_data[i + 1] - y_data[i]);
			}
		}
	}
	return 0.0f;
}

void RVC_TorqueVectoring_run_modeOpen(void)
{
	float32 front_ratio = F_RATIO;
	if (FR_ON)
	{
		float32 u = clamp(RVC.torque.desired / 100.0f, 0.0f, 1.0f);
      
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
		float32 deg = clamp(my_absf(RVC.tv.sta), MIN_STA, MAX_STA);
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

		float32 max_f_ratio = MAX(fl_ratio, fr_ratio);
		float32 max_r_ratio = MAX(rl_ratio, rr_ratio);
		float32 max_ratio = MAX(max_f_ratio, max_r_ratio);

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

	RVC.torque.frontLeft = clamp(RVC.torque.frontLeft, 0.0f, 100.0f);
	RVC.torque.frontRight = clamp(RVC.torque.frontRight, 0.0f, 100.0f);
	RVC.torque.rearLeft = clamp(RVC.torque.rearLeft, 0.0f, 100.0f);
	RVC.torque.rearRight = clamp(RVC.torque.rearRight, 0.0f, 100.0f);

	if (REGEN_ON)
	{
		float32 total_bp = RVC.BrakePressure1.value + RVC.BrakePressure2.value;
		sint16 min_f_rpm = MIN(RVC.AmkMonitor.MotorVelocity.velocity_FL, RVC.AmkMonitor.MotorVelocity.velocity_FR);
		sint16 min_r_rpm = MIN(RVC.AmkMonitor.MotorVelocity.velocity_RL, RVC.AmkMonitor.MotorVelocity.velocity_RR);
		sint16 min_rpm = MIN(min_f_rpm, min_r_rpm);
		sint16 max_f_rpm = MAX(RVC.AmkMonitor.MotorVelocity.velocity_FL, RVC.AmkMonitor.MotorVelocity.velocity_FR);
		sint16 max_r_rpm = MAX(RVC.AmkMonitor.MotorVelocity.velocity_RL, RVC.AmkMonitor.MotorVelocity.velocity_RR);
		float32 max_rpm = (float32)MAX(max_f_rpm, max_r_rpm);
		if (total_bp > 20 && min_rpm > 100)
		{
			RVC.torque.frontLeft = FD * RVC.torque.controlled;
			RVC.torque.frontRight = FD * RVC.torque.controlled;
			RVC.torque.rearLeft = RVC.torque.controlled;
			RVC.torque.rearRight = RVC.torque.controlled;

			RVC.torque.frontLeft = clamp(RVC.torque.frontLeft, -100.0f, 0.0f);
			RVC.torque.frontRight = clamp(RVC.torque.frontRight, -100.0f, 0.0f);
			RVC.torque.rearLeft = clamp(RVC.torque.rearLeft, -100.0f, 0.0f);
			RVC.torque.rearRight = clamp(RVC.torque.rearRight, -100.0f, 0.0f);
		}
	}
}

void RVC_TorqueVectoring_run_mode1(void)
{
	// TODO: TV algorithm
	/*Default*/
	RVC_TorqueVectoring_run_modeOpen();
}
