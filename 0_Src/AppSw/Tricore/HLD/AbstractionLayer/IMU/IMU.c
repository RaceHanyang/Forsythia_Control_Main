#include <IMU.h>
#include <stdbool.h>

#define freeAccId     0x320
#define gyrId         0x420


IMU_t IMU;

/******************* Private Function Prototypes *********************/
void IMU_CAN_init(void);
void IMU_run_1ms(void);
IFX_STATIC void IMU_receiveMessage(void);
IFX_STATIC void Change_Bigendian_to_Littleendian(uint32 msg_data[2], uint32 target_data[2]);

/********************* Function Implementation ***********************/
void IMU_CAN_init(void)
{
	{
		CanCommunication_Message_Config config;
		config.messageId = freeAccId;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&IMU.free_acc_msg, &config);
	}
	{
		CanCommunication_Message_Config config;
		config.messageId = gyrId;
		config.frameType = IfxMultican_Frame_receive;
		config.dataLen = IfxMultican_DataLengthCode_8;
		config.node = &CanCommunication_canNode0;
		config.isStandardId = TRUE;
		CanCommunication_initMessage(&IMU.gyr_msg, &config);
	}
}

void IMU_run_1ms(void)
{
    IMU_receiveMessage();
}

IFX_STATIC void IMU_receiveMessage(void)
{
    if (CanCommunication_receiveMessage(&IMU.free_acc_msg))
	{
		Change_Bigendian_to_Littleendian(IMU.free_acc_msg.msg.data,IMU.free_acc.data);
		//unit conversion
		IMU.IMU_value.free_acc_x_value = IMU.free_acc.s.free_acc_x/(256.0f); 
		IMU.IMU_value.free_acc_y_value = IMU.free_acc.s.free_acc_y/(256.0f);
		IMU.IMU_value.free_acc_z_value = IMU.free_acc.s.free_acc_z/(256.0f);
	}

    if (CanCommunication_receiveMessage(&IMU.gyr_msg))
	{
		Change_Bigendian_to_Littleendian(IMU.gyr_msg.msg.data,IMU.gyr.data);
		//unit conversion
		IMU.IMU_value.gyr_x_value = IMU.gyr.s.gyr_x/(512.0f);
		IMU.IMU_value.gyr_y_value = IMU.gyr.s.gyr_y/(512.0f);
		IMU.IMU_value.gyr_z_value = IMU.gyr.s.gyr_z/(512.0f);
	}
}

IFX_STATIC void Change_Bigendian_to_Littleendian(uint32 msg_data[2], uint32 output_data[2])
{
	typedef union 
	{
		uint64 big_end_data;
		struct 
		{
			uint32 msg_data[2];
		} s;
	} big_end_t;

	typedef union 
	{
		uint64 little_end_data;
		struct 
		{
			uint32 value_data[2];
		} s;
	} little_end_t;

	big_end_t 		  big_end;
	little_end_t 	  little_end;
	boolean temp[64] = {0};

	big_end.s.msg_data[0] = msg_data[0];
	big_end.s.msg_data[1] = msg_data[1];

	for(int i = 0; i < 64; i++)
	{
		temp[i] = ((big_end.big_end_data >> i) & 1ULL);
	}
	for(int i = 63; i >= 0; i--)
	{
		little_end.little_end_data |= (temp[i] << (63-i));
	}

	output_data[0] = little_end.s.value_data[0];
	output_data[1] = little_end.s.value_data[1];
}
