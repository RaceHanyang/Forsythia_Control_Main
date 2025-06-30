#include "Steering_wheel.h"

#define steering_wheel_Id = 0x00030300UL

steering_wheel_t steering_wheel;
CanCommunication_Message steering_wheel_msg;

void SDP_SteeringWheelCan_init(void);
void SDP_SteeringWheel_run_100ms(void);
IFX_STATIC void SDP_SteeringWheel_recieveMessage(void);

void SDP_SteeringWheelCan_init(void)
{
	CanCommunication_Message_Config config;
	config.messageId = steering_wheel_Id;
	config.frameType = IfxMultican_Frame_receive;
	config.dataLen = IfxMultican_DataLengthCode_8;
	config.node = &CanCommunication_canNode0;
	config.isStandardId = FALSE;
	CanCommunication_initMessage(&steering_wheel_msg, &config);
}

void SDP_SteeringWheel_run_100ms(void)
{
    SDP_SteeringWheel_recieveMessage();
}

IFX_STATIC void SDP_SteeringWheel_recieveMessage(void)
{
    CanCommunication_receiveMessage(&steering_wheel_msg);
}