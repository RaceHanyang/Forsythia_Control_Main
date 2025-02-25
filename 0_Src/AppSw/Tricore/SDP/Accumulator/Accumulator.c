/*
 * Accumulator.c
 *
 *  Created on: 2025. 2. 19.
 *      Author: tkdgusqkr
 */

#include "Accumulator.h"

//RX ID~
#define ACCUMULATOR_CAN_MSGBATTERYDIAGNOSE_ID (0x334C01)
#define ACCUMULATOR_CAN_MSGFANSTATUSDATA_ID  (0X334C02)
//#define ACCUMULATOR_CAN_MSGTCORDERECHO_ID (0X334C03)
#define ACCUMULATOR_CAN_MSGFANTARGETDUTY_ID (0X334C04)
//~RX ID

#define ACCUMULATOR_TCORDER_ID (0X275B01)

Accumulator_t Accumulator;

void SDP_Accumulator_init(void) {
	{
        CanCommunication_Message_Config config;
        config.messageId		=	ACCUMULATOR_CAN_MSGBATTERYDIAGNOSE_ID;
        config.frameType		=	IfxMultican_Frame_receive;
        config.dataLen			=	IfxMultican_DataLengthCode_8;
        config.node				=	&CanCommunication_canNode0;
        config.isStandardId		=	FALSE;
        CanCommunication_initMessage(&Accumulator.msgBatteryDiagnose, &config);
	}
	{
        CanCommunication_Message_Config config;
        config.messageId		=	ACCUMULATOR_CAN_MSGFANSTATUSDATA_ID;
        config.frameType		=	IfxMultican_Frame_receive;
        config.dataLen			=	IfxMultican_DataLengthCode_8;
        config.node				=	&CanCommunication_canNode0;
        config.isStandardId		=	FALSE;
        CanCommunication_initMessage(&Accumulator.msgFanStatusData, &config);
	}/*
	{
        CanCommunication_Message_Config config;
        config.messageId		=	ACCUMULATOR_CAN_MSGTCORDERECHO_ID;
        config.frameType		=	IfxMultican_Frame_receive;
        config.dataLen			=	IfxMultican_DataLengthCode_8;
        config.node				=	&CanCommunication_canNode0;
        CanCommunication_initMessage(&A, &config);
	}*/
	{
        CanCommunication_Message_Config config;
        config.messageId		=	ACCUMULATOR_CAN_MSGFANTARGETDUTY_ID;
        config.frameType		=	IfxMultican_Frame_receive;
        config.dataLen			=	IfxMultican_DataLengthCode_8;
        config.node				=	&CanCommunication_canNode0;
        config.isStandardId		=	FALSE;
        CanCommunication_initMessage(&Accumulator.msgFanTargetDuty, &config);
	}
	{
        CanCommunication_Message_Config config;
        config.messageId		=	ACCUMULATOR_TCORDER_ID;
        config.frameType		=	IfxMultican_Frame_transmit;
        config.dataLen			=	IfxMultican_DataLengthCode_8;
        config.node				=	&CanCommunication_canNode0;
        config.isStandardId		=	FALSE;
        CanCommunication_initMessage(&Accumulator.msgTC_order, &config);
	}
}

void SDP_Accumulator_run_10ms(void)
{
    if(CanCommunication_receiveMessage(&Accumulator.msgBatteryDiagnose))
    {
    	Accumulator.BatteryDiagnose.ReceivedData[0] = Accumulator.msgBatteryDiagnose.msg.data[0];
    	Accumulator.BatteryDiagnose.ReceivedData[1] = Accumulator.msgBatteryDiagnose.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&Accumulator.msgFanStatusData))
    {
    	Accumulator.FanStatusData.ReceivedData[0] = Accumulator.msgFanStatusData.msg.data[0];
    	Accumulator.FanStatusData.ReceivedData[1] = Accumulator.msgFanStatusData.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&Accumulator.msgFanTargetDuty))
    {
    	Accumulator.FanTargetDuty.ReceivedData[0] = Accumulator.msgFanTargetDuty.msg.data[0];
    	Accumulator.FanTargetDuty.ReceivedData[0] = Accumulator.msgFanTargetDuty.msg.data[0];
    }

//    CanCommunication_transmitMessage(&Accumulator.msgTC_order);
}

void SDP_Accumulator_setVCUmode(void) {
	Accumulator.TC_order.S.TCControlMode = 1;

	CanCommunication_setMessageData(Accumulator.TC_order.TransmitData[0], Accumulator.TC_order.TransmitData[1], &Accumulator.msgTC_order);
}

void SDP_Accumulator_resetVCUmode(void) {
	Accumulator.TC_order.S.TCControlMode = 0;

	CanCommunication_setMessageData(Accumulator.TC_order.TransmitData[0], Accumulator.TC_order.TransmitData[1], &Accumulator.msgTC_order);
}

IFX_STATIC int normalize(uint8 percentage) {
	if(percentage < 0) return 0;
	if(percentage > 100) return 100;
	return percentage;
}

void SDP_Accumulator_setFanDuty(uint8 sideIntakePercentage, uint8 segmentIntakePercentage, uint8 segmentExhaust60Percentage, uint8 segmentExhaust80Percentage) {
	sideIntakePercentage = normalize(sideIntakePercentage);
	segmentIntakePercentage = normalize(segmentIntakePercentage);
	segmentExhaust60Percentage = normalize(segmentExhaust60Percentage);
	segmentExhaust80Percentage = normalize(segmentExhaust80Percentage);
	//ZERI
	Accumulator.TC_order.S.TCFanDutyOrder_SideIntake = sideIntakePercentage;
	Accumulator.TC_order.S.TCFanDutyOrder_SegmentIntake70 = segmentIntakePercentage;
	Accumulator.TC_order.S.TCFanDutyOrder_SegmentExhaust60 = segmentExhaust60Percentage;
	Accumulator.TC_order.S.TCFanDutyOrder_SegmentExhaust80 = segmentExhaust80Percentage;

	CanCommunication_setMessageData(Accumulator.TC_order.TransmitData[0], Accumulator.TC_order.TransmitData[1], &Accumulator.msgTC_order);
}
