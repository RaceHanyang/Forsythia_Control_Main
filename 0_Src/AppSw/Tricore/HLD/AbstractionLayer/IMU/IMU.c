#include <IMU.h>

#define free_acc_Id     0x320
#define gyr_Id          0x420


IMU_t IMU;

/******************* Private Function Prototypes *********************/
void IMU_CAN_init(void);
void IMU_run_1ms(void);
IFX_STATIC void IMU_receiveMessage(void);

/********************* Function Implementation ***********************/
void IMU_CAN_init(void)
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

void IMU_run_1ms(void)
{
    IMU_receiveMessage();
}

IFX_STATIC void IMU_receiveMessage(void)
{
    if (CanCommunication_receiveMessage(&new_IMU.free_acc_msg))
	{
        IMU.free_acc.data[0] = IMU.free_acc_msg.msg.data[0];
        IMU.free_acc.data[1] = IMU.free_acc_msg.msg.data[1];
		{ //unit conversion
			IMU.IMU_value.free_acc_x_value = IMU.free_acc.s.free_acc_x/(256); 
			IMU.IMU_value.free_acc_y_value = IMU.free_acc.s.free_acc_y/(256);
			IMU.IMU_value.free_acc_z_value = IMU.free_acc.s.free_acc_z/(256);
		}
	}
    if (CanCommunication_receiveMessage(&new_IMU.gyr_msg))
	{
        IMU.gyr.data[0] = IMU.gyr_msg.msg.data[0];
        IMU.gyr.data[1] = IMU.gyr_msg.msg.data[1];
		{ //unit conversion
			IMU.IMU_value.gyr_x_value = IMU.gyr.s.gyr_x/(512);
			IMU.IMU_value.gyr_y_value = IMU.gyr.s.gyr_y/(512);
			IMU.IMU_value.gyr_z_value = IMU.gyr.s.gyr_z/(512);
		}
	}
}