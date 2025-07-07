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
void New_IMU_run_10ms(void);
IFX_STATIC void New_IMU_receiveMessage(void);

/********************* Function Implementation ***********************/
void New_IMU_CAN_init(void)
{
	{
		CanCommunication_Message_Config config;
		config.messageId = utc_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.utc_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = imu_err_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.imu_err_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = imu_status_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.imu_status_msg, &config);
	}
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
		config.messageId = acc_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.acc_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = velocity_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.velocity_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = euler_ang_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.euler_angle_msg, &config);
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
	{
		CanCommunication_Message_Config config;
		config.messageId = coordi_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.coordi_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = altitude_Id;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&new_IMU.altitude_msg, &config);
	}
}

void New_IMU_run_10ms(void)
{
    New_IMU_receiveMessage();
}

IFX_STATIC void New_IMU_receiveMessage(void)
{
	if (CanCommunication_receiveMessage(&new_IMU.utc_msg))
	{
		new_IMU.utc.data[0] = new_IMU.utc_msg.msg.data[0];
		new_IMU.utc.data[1] = new_IMU.utc_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.imu_err_msg))
	{
		new_IMU.imu_err.data[0] = new_IMU.imu_err_msg.msg.data[0];
		new_IMU.imu_err.data[1] = new_IMU.imu_err_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.free_acc_msg))
	{
        new_IMU.free_acc.data[0] = new_IMU.free_acc_msg.msg.data[0];
        new_IMU.free_acc.data[1] = new_IMU.free_acc_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.acc_msg))
	{
        new_IMU.acc.data[0] = new_IMU.acc_msg.msg.data[0];
        new_IMU.acc.data[1] = new_IMU.acc_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.velocity_msg))
	{
        new_IMU.velocity.data[0] = new_IMU.velocity_msg.msg.data[0];
        new_IMU.velocity.data[1] = new_IMU.velocity_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.euler_angle_msg))
	{
        new_IMU.euler_angle.data[0] = new_IMU.euler_angle_msg.msg.data[0];
        new_IMU.euler_angle.data[1] = new_IMU.euler_angle_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.gyr_msg))
	{
        new_IMU.gyr.data[0] = new_IMU.gyr_msg.msg.data[0];
        new_IMU.gyr.data[1] = new_IMU.gyr_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.coordi_msg))
	{
        new_IMU.coordi.data[0] = new_IMU.coordi_msg.msg.data[0];
        new_IMU.coordi.data[1] = new_IMU.coordi_msg.msg.data[1];
	}
    if (CanCommunication_receiveMessage(&new_IMU.altitude_msg))
	{
        new_IMU.altitude.data[0] = new_IMU.altitude_msg.msg.data[0];
        new_IMU.altitude.data[1] = new_IMU.altitude_msg.msg.data[1];
    }
}