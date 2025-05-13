/*
 * OrionBms2.c
 * Created on: 2020.08.04
 * Author: Dua
 */

/***************************** Includes ******************************/
#include <IfxCpu.h>

#include "OrionBms2.h"
#include "RVC.h"
/**************************** Macro **********************************/

/*********************** Global Variables ****************************/
const uint32 pack_pow_Id = 0x00000300UL;
const uint32 pack_status_Id = 0x00020000UL;
const uint32 pack_cv_Id = 0x00040000UL;
const uint32 pack_ocv_Id = 0x00040001UL;
const uint32 pack_temp_Id = 0x00040002UL;
const uint32 pack_soc_Id = 0x00040003UL;

OrionBms2_t OrionBms2;

/******************* Private Function Prototypes *********************/
IFX_STATIC void OrionBms2_receiveMessage(void);

/********************* Function Implementation ***********************/
void OrionBms2_init(void)
{
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_pow_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&OrionBms2.pack_pow_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_status_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = FALSE;
		CanCommunication_initMessage(&OrionBms2.pack_status_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_cv_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = FALSE;
		CanCommunication_initMessage(&OrionBms2.pack_cv_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_ocv_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&OrionBms2.pack_ocv_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_temp_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = FALSE;
		CanCommunication_initMessage(&OrionBms2.pack_temp_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = pack_soc_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = FALSE;
		CanCommunication_initMessage(&OrionBms2.pack_soc_msg, &config);
	}
}

void OrionBms2_run_1ms_c2(void)
{
	OrionBms2_receiveMessage();
}

/****************** Private Function Implementation ******************/

/* TODO:
 * CRC check
 * Update flag
 */
IFX_STATIC void OrionBms2_receiveMessage(void)
{
	if (CanCommunication_receiveMessage(&OrionBms2.pack_pow_msg))
	{
		OrionBms2.pack_pow.data[0] = OrionBms2.pack_pow_msg.msg.data[0];
		OrionBms2.pack_pow.data[1] = OrionBms2.pack_pow_msg.msg.data[1];
		while(IfxCpu_acquireMutex(&RVC_public.bms.shared.mutex))
			; // Wait for mutex
		{
			RVC_public.bms.shared.data.current = (float32)OrionBms2.pack_pow.s.current / 10;
			RVC_public.bms.shared.data.voltage = (float32)OrionBms2.pack_pow.s.voltage / 10;
			RVC_public.bms.shared.data.chargeLimit = OrionBms2.pack_pow.s.ccl;
			RVC_public.bms.shared.data.dischargeLimit = OrionBms2.pack_pow.s.dcl;
			IfxCpu_releaseMutex(&RVC_public.bms.shared.mutex);
		}
	}
	/*
	if (CanCommunication_receiveMessage(&OrionBms2.pack_status_msg))
	{
		OrionBms2.pack_status.data[0] = OrionBms2.pack_status_msg.msg.data[0];
		OrionBms2.pack_status.data[1] = OrionBms2.pack_status_msg.msg.data[1];
	}
	if (CanCommunication_receiveMessage(&OrionBms2.pack_cv_msg))
	{
		OrionBms2.pack_cv.data[0] = OrionBms2.pack_cv_msg.msg.data[0];
		OrionBms2.pack_cv.data[1] = OrionBms2.pack_cv_msg.msg.data[1];
	}
	if (CanCommunication_receiveMessage(&OrionBms2.pack_ocv_msg))
	{
		OrionBms2.pack_ocv.data[0] = OrionBms2.pack_ocv_msg.msg.data[0];
		OrionBms2.pack_ocv.data[1] = OrionBms2.pack_ocv_msg.msg.data[1];
	}
	if (CanCommunication_receiveMessage(&OrionBms2.pack_temp_msg))
	{
		OrionBms2.pack_temp.data[0] = OrionBms2.pack_temp_msg.msg.data[0];
		OrionBms2.pack_temp.data[1] = OrionBms2.pack_temp_msg.msg.data[1];
	}
	if (CanCommunication_receiveMessage(&OrionBms2.pack_soc_msg))
	{
		OrionBms2.pack_soc.data[0] = OrionBms2.pack_soc_msg.msg.data[0];
		OrionBms2.pack_soc.data[1] = OrionBms2.pack_soc_msg.msg.data[1];
	}
	*/
}
