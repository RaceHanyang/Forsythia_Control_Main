/*
 * MechMsg.c
 *
 *  Created on: 2025. 2. 25.
 *      Author: tkdgusqkr
 */

/***************************** Includes ******************************/
#include <IfxCpu.h>

#include "MechMsg.h"
#include "RVC.h"
/**************************** Macro **********************************/

/*********************** Global Variables ****************************/
const uint32 steering_and_pedal_id = 0x00040300UL;

mech_msg_t mech_msg;
mech_msg_public_t mech_msg_public;

/******************* Private Function Prototypes *********************/

/********************* Function Implementation ***********************/
void SDP_MechMsg_init(void)
{
	{
		CanCommunication_Message_Config config;
		config.messageId = steering_and_pedal_id;
		config.frameType = IfxMultican_Frame_transmit;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = FALSE;
		CanCommunication_initMessage(&mech_msg.steering_and_pedal_msg, &config);
	}
}

void SDP_MechMsg_run_10ms(void)
{
	while(IfxCpu_acquireMutex(&mech_msg_public.shared.mutex))
		; // Wait for the mutex
	{
		mech_msg_public.data = mech_msg_public.shared.data;
		IfxCpu_releaseMutex(&mech_msg_public.shared.mutex);
	}
	mech_msg.steering_and_pedal.s.steering_angel = mech_msg_public.data.steering_angel;
	mech_msg.steering_and_pedal.s.apps = mech_msg_public.data.apps;
	mech_msg.steering_and_pedal.s.bpps = mech_msg_public.data.bpps;
	mech_msg.steering_and_pedal.s.brake_pressure_0 = mech_msg_public.data.brake_pressure_0;
	mech_msg.steering_and_pedal.s.brake_pressure_1 = mech_msg_public.data.brake_pressure_1;
	CanCommunication_setMessageData(mech_msg.steering_and_pedal.data[0], mech_msg.steering_and_pedal.data[1], &mech_msg.steering_and_pedal_msg);
	CanCommunication_transmitMessage(&mech_msg.steering_and_pedal_msg);
}

/****************** Private Function Implementation ******************/

