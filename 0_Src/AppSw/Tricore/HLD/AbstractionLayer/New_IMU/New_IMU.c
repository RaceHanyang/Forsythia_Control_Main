#include <New_IMU.h>

#define utc_Id          0x001
#define imu_err_Id      0x120
#define imu_status_Id   0x220
#define free_acc_Id     0x320
#define acc_Id          0x321
#define velocity_Id     0x322
#define euler_ang_Id    0x323
#define gyr_Id          0x420
#define coordi_Id       0x421
#define altitude_Id     0x422

new_IMU_t new_IMU;

/******************* Private Function Prototypes *********************/
void New_IMU_CAN_init(void);
void New_IMU_run_1ms(void);
IFX_STATIC void New_IMU_receiveMessage(void);

/********************* Function Implementation ***********************/
void New_IMU_CAN_init(void)
{
	{
		CanCommunication_Message_Config config;
		config.messageId = free_acc_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.free_acc_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = gyr_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.gyr_msg, &config);
	}
}

void New_IMU_run_1ms(void)
{
    New_IMU_receiveMessage();
}

IFX_STATIC void New_IMU_receiveMessage(void)
{
    if (CanCommunication_receiveMessage(&new_IMU.free_acc_msg))
	{
        new_IMU.free_acc.data[0] = new_IMU.free_acc_msg.msg.data[0];
        new_IMU.free_acc.data[1] = new_IMU.free_acc_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.gyr_msg))
	{
        new_IMU.gyr.data[0] = new_IMU.gyr_msg.msg.data[0];
        new_IMU.gyr.data[1] = new_IMU.gyr_msg.msg.data[1];
	}
}