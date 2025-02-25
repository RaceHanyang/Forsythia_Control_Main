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

void SDP_MechMsg_run_1000ms(void)
{
	CanCommunication_setMessageData(mech_msg.steering_and_pedal.data[0], mech_msg.steering_and_pedal.data[1], &mech_msg.steering_and_pedal_msg);
	CanCommunication_transmitMessage(&mech_msg.steering_and_pedal_msg);
}

/****************** Private Function Implementation ******************/

