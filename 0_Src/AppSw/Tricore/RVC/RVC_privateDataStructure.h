/*
 * RVC_privateDataStructure.h
 * Created on 2019. 12. 10.
 * Author: Dua
 */

#ifndef RVC_PRIVATEDATASTRUCTURE_H_
#define RVC_PRIVATEDATASTRUCTURE_H_

/******************************** Includes ***********************************/
#include "AdcSensor.h"
#include "AmkInverter_can.h"
#include "Gpio_Debounce.h"
#include "HLD.h"

/****************************** Enumerations *********************************/
typedef enum
{
	RVC_ReadyToDrive_status_notInitialized = 0,
	RVC_ReadyToDrive_status_initialized = 1,
	RVC_ReadyToDrive_status_run = 2,
} RVC_ReadyToDrive_status;

typedef enum
{
	RVC_TorqueVectoring_modeOpen = 0,
	RVC_TorqueVectoring_mode1 = 1,
	RVC_TorqueVectoring_diff = 2,
} RVC_TorqueVectoring_mode_t;

typedef enum
{
	RVC_TractionControl_modeNone = 0,
	RVC_TractionControl_mode1 = 1,
} RVC_TractionControl_mode_t;

typedef struct
{
	float32 mul;
	float32 offset;
} RVC_pwmCalibration;

typedef struct
{
	Gpio_Debounce_input debounce;
	boolean value;
} RVC_Gpi_t;

/*************************** Data Structures *********************************/
typedef struct
{
	RVC_ReadyToDrive_status readyToDrive;

	AdcSensor LvBattery_Voltage;
	AdcSensor BrakePressure1;
	AdcSensor BrakePressure2;

	RVC_TorqueVectoring_mode_t tvMode;
	RVC_TractionControl_mode_t tcMode;

	Gpio_Debounce_input startButton;

	RVC_Gpi_t airPositive;
	RVC_Gpi_t airNegative;
	RVC_Gpi_t brakePressureOn;
	RVC_Gpi_t brakeSwitch;
	RVC_Gpi_t tsalOn;
	RVC_Gpi_t sdcSenBspd;
	RVC_Gpi_t sdcSenImd;
	RVC_Gpi_t sdcSenAms;
	RVC_Gpi_t sdcSenFinal;
	RVC_Gpi_t bmsOk;
	RVC_Gpi_t imdOk;
	RVC_Gpi_t bspdOk;
	RVC_Gpi_t bmsMpo;
	RVC_Gpi_t chargeEn;

	struct
	{
		boolean bp1;
		boolean bp2;
		boolean tot;
	} brakeOn;

	struct
	{
		float32 value;
		float32 limit;
		float32 currentLimit;
	} power;

	struct
	{
		float32 value;
		float32 margin;
		float32 setValue;
		boolean isLimited;
	} currentLimit;

	struct
	{
		boolean isAppsChecked;
		boolean isBppsChecked1;
		boolean isBppsChecked2;
	} R2d;

	struct
	{
		float32 desired;
		float32 controlled;

		signed short frontLeft;
		signed short frontRight;
		signed short rearLeft;
		signed short rearRight;

		boolean isRegenOn;
	} torque;

	struct
	{
		HLD_GtmTom_Pwm accel_rearLeft;
		HLD_GtmTom_Pwm accel_rearRight;
		HLD_GtmTom_Pwm decel_rearLeft;
		HLD_GtmTom_Pwm decel_rearRight;
	} out;

	struct
	{
		RVC_pwmCalibration leftAcc;
		RVC_pwmCalibration rightAcc;
		RVC_pwmCalibration leftDec;
		RVC_pwmCalibration rightDec;
	} calibration; // FIXME: To be suitable for LTC2645

	struct Monitor AmkMonitor;

	struct
	{
		float32 sta;
	} tv;

	struct
	{
		double v[2][2]; // 2x2 matrix
		double a_x;     // Acceleration in x direction

		double v_x;   // Velocity in x direction
		double v_ref; // Reference velocity in x direction
	} v_x;

	struct
	{
		struct
		{
			double u;   // pedal input
			double v_x; // Velocity in x direction

			double a_ff; // Alpha of feed forward
		} ff;

		struct
		{
			double v; // voltage of pack
			double i; // current of pack
			double p; // power of pack

			double a_fb; // Alpha of feed back
		} fb;

		double f_des; // Desired force in x direction
	} pwr_lim;

	struct
	{
		struct
		{
			struct
			{
				double u; // Pedal input

				double k_0; // Gain in base
			} base;

			struct
			{
				double a_x;

				double k_a; // Gain in acceleration
			} acc;

			struct
			{
				double r;     // yaw rate
				double r_des; // Desired yaw rate

				double k_r; // Gain in yaw rate
			} yaw;

			double f_des; // Desired force in x direction

			double k_fr; // Gain in front rear direction
			double f_f;  // Force in front direction
			double f_r;  // Force in rear direction

			double k_frl;   // limit saturation
			double f_r_sat; // Saturated force
		} f_r;

		struct
		{
			double m_c; // target yaw moment
			double f_f; // Force in front direction
			double f_r; // Force in rear direction

			double k_tfr;

			double f_fl; // Force in front left direction
			double f_fr; // Force in front right direction
			double f_rl; // Force in rear left direction
			double f_rr; // Force in rear right direction
		} l_r;
	} torque_vectoring;

	uint16 RTDS_Tick;

} RVC_t;

#endif
