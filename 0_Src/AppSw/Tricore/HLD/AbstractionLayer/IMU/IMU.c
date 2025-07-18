#include <IMU.h>


#define freeAccId     0x320
#define gyrId         0x420


IMU_t IMU;

/******************* Private Function Prototypes *********************/
void IMU_CAN_init(void);
void IMU_run_1ms(void);
IFX_STATIC void IMU_receiveMessage(void);
IFX_STATIC void Change_Bigendian_to_Littleendian(uint32 input[2], uint32 output[2]);

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

IFX_STATIC void Change_Bigendian_to_Littleendian(uint32* input, uint32* output)
{
	typedef union
	{
		uint32 data[2];
		uint8 input_temp[8]; 
	}input_to_temp_t; //Divied into 1Byte

	typedef union 
	{
		uint32 data[2];
		uint8 output_temp[8];
	}temp_to_output_t; 

	input_to_temp_t  input_to_temp;
	temp_to_output_t temp_to_output;

	input_to_temp.data[0] = input[0];
	input_to_temp.data[1] = input[1];

	temp_to_output.output_temp[0] = input_to_temp.input_temp[1];
	temp_to_output.output_temp[1] = input_to_temp.input_temp[0];
	temp_to_output.output_temp[2] = input_to_temp.input_temp[3];
	temp_to_output.output_temp[3] = input_to_temp.input_temp[2];
	temp_to_output.output_temp[4] = input_to_temp.input_temp[5];
	temp_to_output.output_temp[5] = input_to_temp.input_temp[4];
	temp_to_output.output_temp[6] = input_to_temp.input_temp[7];
	temp_to_output.output_temp[7] = input_to_temp.input_temp[6];  
	
	output[0] = temp_to_output.data[0];
	output[1] = temp_to_output.data[1];
}
