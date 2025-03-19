/*
 * PedalBox.h
 *
 *  Created on: 2021. 09. 29.
 *      Author: Suprhimp
 */

#include "SDP.h"
#include "SteeringAngleAdc.h"
#include "GtmTim.h"
#include <math.h>

#define SSTROKE		(60.0f)
#define SSTT		(0.5f) // min = 0.5
#define SEND		(4.5f) // max = 4.5

AdcSensor STA0;	//	left
AdcSensor STA1;	//	right

SDP_SteeringAngleAdc_angle_t 	SDP_SteeringAngleAdc_angle;
SDP_SteeringAngleAdc_t 			SDP_SteeringAngleAdc;

void SDP_SteeringAngleAdc_init(void);
IFX_STATIC void SDP_SteeringAngleAdc_updateSTA_AN(SDP_SteeringAngleAdc_sensor_t *data_out, AdcSensor *data_in);
void SDP_SteeringAngleAdc_run(void);


void SDP_SteeringAngleAdc_init(void){
        AdcSensor_Config config_adc;
		//STA0
		config_adc.adcConfig.lpf.config.cutOffFrequency = 10000/(2.0*IFX_PI*0.05);		//FIXME: Adjust time constant
		config_adc.adcConfig.lpf.config.gain = 1;
		config_adc.adcConfig.lpf.config.samplingTime = 0.001;
		config_adc.adcConfig.lpf.activated = TRUE;

		config_adc.adcConfig.channelIn = &HLD_Vadc_P10_7_G3CH0_AD5;
		config_adc.tfConfig.a = SSTROKE / (SEND - SSTT);
		config_adc.tfConfig.b = config_adc.tfConfig.a * (-SSTT);

		config_adc.isOvervoltageProtected = FALSE;

		AdcSensor_initSensor(&STA0, &config_adc);
		HLD_AdcForceStart(STA0.adcChannel.channel.group);

		//STA1
		config_adc.adcConfig.channelIn = &HLD_Vadc_P33_10_G5CH4_DA0;
		AdcSensor_initSensor(&STA1, &config_adc);
		HLD_AdcForceStart(STA1.adcChannel.channel.group);

		SDP_SteeringAngleAdc_angle.sta0.config.radius = 14.0f;
		SDP_SteeringAngleAdc_angle.sta0.config.neutral = 0.0f;

		SDP_SteeringAngleAdc_angle.sta0.config.radius = 14.0f;
		SDP_SteeringAngleAdc_angle.sta0.config.neutral = 0.0f;
}


IFX_STATIC void SDP_SteeringAngleAdc_updateSTA_AN(SDP_SteeringAngleAdc_sensor_t *data_out, AdcSensor *data_in)
{
	AdcSensor_getData(data_in);
	// data_out->pedalPercent = data_out->config.reversed
			// ?(float32)100.0 - data_in->value : data_in->value;
	data_out->value	 = (data_in->value);
	data_out->radian = (data_out->config.neutral - data_in->value) / data_out->config.radius;
}

IFX_STATIC void SDP_SteeringAngleAdc_checkErrorState_fromTwo(SDP_SteeringAngleAdc_sensor_t *data1, SDP_SteeringAngleAdc_sensor_t *data2)
{
	float32 diff = fabs(data1->radian) - fabs(data2->radian);
	float32 absDiff = fabs(diff);

	if (absDiff>10)
	{
		data1 -> isValueOk = FALSE;
		data2 -> isValueOk = FALSE;
	}
	else
	{
		data1 -> isValueOk = TRUE;
		data2 -> isValueOk = TRUE;
	}

	if (data1->value < 0 || data1->value > SSTROKE)
	{
		data1 -> isValueOk = FALSE;
	}
	if (data2->value < 0 || data2->value > SSTROKE)
	{
		data2 -> isValueOk = FALSE;
	}
}

void SDP_SteeringAngleAdc_run(){
	uint8 	okCount = 0;
	float32 sum = 0;

    SDP_SteeringAngleAdc_updateSTA_AN(&SDP_SteeringAngleAdc_angle.sta0,&STA0);
	SDP_SteeringAngleAdc_updateSTA_AN(&SDP_SteeringAngleAdc_angle.sta1,&STA1);

	SDP_SteeringAngleAdc_checkErrorState_fromTwo(&SDP_SteeringAngleAdc_angle.sta0 , &SDP_SteeringAngleAdc_angle.sta1);

	if (SDP_SteeringAngleAdc_angle.sta0.isValueOk)
	{
		sum -= SDP_SteeringAngleAdc_angle.sta0.radian;
		okCount++;
	}

	if (SDP_SteeringAngleAdc_angle.sta1.isValueOk)
	{
		sum += SDP_SteeringAngleAdc_angle.sta1.radian;
		okCount++;
	}

	if (okCount >= 2)
	{
		SDP_SteeringAngleAdc.sta.radian = (float32)sum / (float32)okCount;
		SDP_SteeringAngleAdc.sta.degree = SDP_SteeringAngleAdc.sta.radian * (180.0f / IFX_PI);
		SDP_SteeringAngleAdc.sta.isValueOk = TRUE;
	}
	else
	{
		SDP_SteeringAngleAdc.sta.isValueOk = FALSE;
	}
}
