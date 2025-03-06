/*
 * SteeringAngleAdc.h
 *
 *  Created on: 2021. 09. 29.
 *      Author: Suprhimp
 */

#ifndef SRC_APPSW_TRICORE_SDP_SteeringAngleAdc_H_
#define SRC_APPSW_TRICORE_SDP_SteeringAngleAdc_H_
#include "GtmTim.h"
#include "SDP.h"
#include "AdcSensor.h"

typedef struct
{
	float32						radius;		//	rack gear radius, mm
	float32						neutral;	//	neutral position, mm

}SDP_SteeringAngleAdc_sensorConfig_t;

typedef struct
{
	SDP_SteeringAngleAdc_sensorConfig_t	config;
	float32								value;
	float32								radian;
	boolean								isValueOk;
}SDP_SteeringAngleAdc_sensor_t;

typedef struct
{
	SDP_SteeringAngleAdc_sensor_t		sta0;
	SDP_SteeringAngleAdc_sensor_t		sta1;
}SDP_SteeringAngleAdc_angle_t;

typedef struct
{
	float32	radian;
	float32	degree;
	boolean	isValueOk;
}SDP_SteeringAngleAdc_struct_t;

typedef struct
{
	SDP_SteeringAngleAdc_struct_t			sta;
}SDP_SteeringAngleAdc_t;

IFX_EXTERN SDP_SteeringAngleAdc_sensor_t SDP_SteeringAngleAdc_sensor;
IFX_EXTERN void SDP_SteeringAngleAdc_init(void);
IFX_EXTERN void SDP_SteeringAngleAdc_run(void);

#endif
