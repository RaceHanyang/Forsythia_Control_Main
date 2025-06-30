/*includes*/
#include <IfxCpu.h>
#include "Steering_wheel.h"

/*defines*/
#define steering_wheel_Id = 0x00030300UL

/*structures*/
steering_wheel_t steering_wheel;

/*function prototypes*/
void SteeringWheelCan_init(void);
void SteeringWheel_run_100ms(void);
IFX_STATIC void SteeringWheel_receiveMessage(void);

/*function implimentations*/
void SteeringWheelCan_init(void)
{
	CanCommunication_Message_Config config;
	config.messageId = 0x00030300UL;
	config.frameType = IfxMultican_Frame_receive;
	config.dataLen = IfxMultican_DataLengthCode_8;
	config.node = &CanCommunication_canNode0;
	config.isStandardId = FALSE;
	CanCommunication_initMessage(&steering_wheel_msg, &config);
}


void SteeringWheel_run_100ms(void)
{
    SteeringWheel_receiveMessage();
}

IFX_STATIC void SteeringWheel_receiveMessage(void)
{
    if(CanCommunication_receiveMessage(&steering_wheel_msg) == TRUE)
    {
    	steering_wheel.data[0] = steering_wheel_msg.msg.data[0];
        steering_wheel.data[1] = steering_wheel_msg.msg.data[1];
    }
}
