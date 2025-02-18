/*
 * Accumulator.h
 *
 *  Created on: 2025. 2. 19.
 *      Author: tkdgusqkr
 */

#ifndef __INC_ACCUMULATOR_H__
#define __INC_ACCUMULATOR_H__

#include "SDP.h"
#include "CanCommunication.h"
#include "HLD.h"

/*
 * RX Data
 * Receive From: Accumulator Box
 * RX ID: 0x334C01
 * Data: Precharge state1&2, relaycontact signal 1&2&3, TSAL signal, IMD status
 */
typedef union{
	uint32 ReceivedData[8];
	struct{
		uint8 prechargeStateSignal1 ;
		uint8 prechargeStateSignal2 ;
		uint8 RelayContactSignal1   ;
		uint8 RelayContactSignal2	  ;
		uint8 RelayContactSignal3	  ;
		uint8 TsalSignal			  ;
		uint8 IMDStatusFrequency	  ;
		uint8 Reserved			  ;


	}S;
}BatteryDiagnose_t;

/*
 * RX Data
 * Receive From: VCU
 * RX ID: 0x334C02
 * Data: Fan Flag, TIM 15, 16, 17 Dutycycle & Frequency
 *
 ***Notation
 * Ex) if fanflag == 1, send frequency&duty cycle of no.7,4,1 fans.
 * Fanflag = 1: tim15: 7
 * 			 tim16: 4
 * 			 tim17: 1
 * Fanflag = 2: tim15: 8
 * 			 tim16: 5
 * 			 tim17: 2
 * Fanflag = 3: tim15: 9
 * 			 tim16: 6
 * 			 tim17: 3
 */
typedef union{
	uint32 ReceivedData[2];
	struct{
		uint8 FanFlag			;
		uint8 TIM15_Dutycycle ;
		uint8 TIM15_Frequency ;
		uint8 TIM16_Dutycycle ;
		uint8 TIM16_Frequency ;
		uint8 TIM17_Dutycycle ;
		uint8 TIM17_Frequency ;
		uint8 desiredDuty			;
	}S;

}FanStatusData_t;

/*
 * TX Data
 * Transmit To: VCU
 * TX ID: 0x275B01
 * Data: TCControl mode 0/1, one TCFanDutyOrder for each of 4 segments, in range of 0~100.
 */
typedef union{
	union{
		uint32 TransmitData[2];
	};
	struct{
		uint8 TCControlMode	; //0: not TC control mode, 1: TC Control mode
		uint8 TCFanDutyOrder_SideIntake ;
		uint8 TCFanDutyOrder_SegmentIntake70;
		uint8 TCFanDutyOrder_SegmentExhaust60;
		uint8 TCFanDutyOrder_SegmentExhaust80;
		uint8 Remain1		;
		uint8 Remain2		;
		uint8 Remain3		;

	}S;

}TC_order_t;

/*
 * RX Data
 * Receive From: Accumulator
 * RX ID: 0x334C04
 * Data: Transmit desired duty for 4 fan clusters, in range of 0 to 100.
 */
typedef union{
	uint32 ReceivedData[2];
	struct{
		uint8 TargetDuty_SideIntake ;
		uint8 TargetDuty_SegmentIntake70;
		uint8 TargetDuty_SegmentExhaust60;
		uint8 TargetDuty_SegmentExhaust80;
		uint8 Remain1;
		uint8 Remain2;
		uint8 Remain3;
		uint8 Remain4;
	}S;
}FanTargetDuty_t;

typedef struct{
	BatteryDiagnose_t BatteryDiagnose;
	FanStatusData_t FanStatusData;
	TC_order_t TC_order;
	FanTargetDuty_t FanTargetDuty;

	CanCommunication_Message msgBatteryDiagnose;
	CanCommunication_Message msgFanStatusData;
	CanCommunication_Message msgTC_order;
	CanCommunication_Message msgFanTargetDuty;
}Accumulator_t;

IFX_EXTERN Accumulator_t Accumulator;


#endif
