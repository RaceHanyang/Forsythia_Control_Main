/*
 * RVC.h
 * Created on 2019. 11. 01
 * Author: Dua
 */

#ifndef RVC_H_
#define RVC_H_

/* Includes */
#include "SDP.h"

/* Data Structures */
typedef struct
{
	float32 current;
	float32 voltage;
	uint16 chargeLimit;
	uint16 dischargeLimit;

	IfxCpu_mutexLock mutex;
	boolean isUpdated;
} RVC_public_bms_t;

typedef struct
{
	uint32 rpm;
	float32 current;
	float32 voltage;
	uint32 inveterTemp;
	uint32 controllerTemp;
} RVC_public_inverter_t;

typedef struct 
{	
	float32 alpha_ff;	//feed forward
    float32 alpha_fb;	//feed back

	float32 alpha;

	struct 
	{
		const float32   V_epsilon = (0.001f); 	//Unit : M/s
		const uint16    F_max = 2143;			//Unit : 10mN/M
		const float32	K_p = ((1.0f)/5);		//Gain, 1/P_band (P_band : Linear decay range (Uint : kW))
		const uint16 	T_s	= 1000;				//Sampling Period    (Uint : Hz)
		const float32   t = (0.05f);			//Time constant      (Unit : sec)
	} constants;
} RVC_alpha_t;




typedef struct
{
	struct
	{
		RVC_public_bms_t data;

		struct
		{
			RVC_public_bms_t data;
			IfxCpu_mutexLock mutex;
			boolean isUpdated;
		} shared;
	} bms;

	struct
	{
		RVC_public_inverter_t data;

		struct
		{
			RVC_public_inverter_t data;
			IfxCpu_mutexLock mutex;
			boolean isUpdated;
		} shared;
	} inverter1, inverter2;
	
	RVC_alpha_t RVC_alpha;
} RVC_public_t;

/* Global Variables */
IFX_EXTERN RVC_public_t RVC_public;

/* Function Prototypes */
IFX_EXTERN void RVC_init(void);
IFX_EXTERN void RVC_run_1ms(void);
IFX_EXTERN void RVC_run_10ms(void);

#endif
