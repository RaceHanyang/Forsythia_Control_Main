#include "AmkInverter_can.h"
#include "HLD.h"

#if AMK_MODE == 0
#define AMK_VELOCITY_LIM  			18000
#define AMK_VELOCITY_STOP  			0
#define AMK_TORQUE_POSITIVE_LIM  	2143
#define AMK_TORQUE_NEGATIVE_LIM  	-2143

#define MAX_AMK_ERROR_RESET			10

#define AMK_RESTART_ERROR			3587

#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_RL 0
#define MOTOR_RR 3

private_inv_t			inv[4];
private_inv_seq_t 		inv_seq;
private_inv_status_t 	inv_status;

struct Monitor Monitor;

AmkInverterPublic_t AmkInverterPublic;
AmkInverterMonitorPublic_t AmkInverterMonitorPublic;

void AmkInverter_can_init(void);
void AmkInverter_can_Run(void);
void AmkInverter_Start(boolean rtd_flag);
void AmkInverter_writeMessageFront(sint16 torque_left, sint16 torque_right, boolean accelerating);
void AmkInverter_writeMessageRear(sint16 torque_left, sint16 torque_right, boolean accelerating);

static void SetId(id_set_t *id, int node);
static void setTransmitMessage(uint32 id, CanCommunication_Message *Tm, uint8 node, boolean is_standard_id);
static void setReceiveMessage(uint32 id, CanCommunication_Message *Rm, uint8 node, boolean is_standard_id);
static void seqSet(int i);
static void seqReset(int i);
static void invWrite(int i, sint16 torque, boolean accelerating);
static void switchWrite(void);

void AmkInverter_can_init(void)
{
	inv[0].inv_address = 1;	// inv1
	inv[1].inv_address = 2;	// inv2
	inv[2].inv_address = 5;	// inv3
	inv[3].inv_address = 6;	// inv4

	for(int i = 0; i < 4; i++)
	{
		SetId(&inv[i].inv_id, inv[i].inv_address);
	}

	inv_seq.id 		= 0x010;
	inv_status.id 	= 0x110;

	for(int i = 0; i < 4; i++)
	{
		inv[i].inv_id_log.amk_set 	= 0x40100 + 3 * i;
		inv[i].inv_id_log.amk_ac1 	= 0x40101 + 3 * i;
		inv[i].inv_id_log.amk_ac2 	= 0x40102 + 3 * i;
	}

	for (int i = 0; i < 4; i++)
	{
		if (i == MOTOR_FL || i == MOTOR_FR)
		{
			setTransmitMessage(inv[i].inv_id.amk_set, &inv[i].t_amk_setpoint_1, 2, TRUE);
			setReceiveMessage(inv[i].inv_id.amk_ac1, &inv[i].r_amk_actual_values_1, 2, TRUE);
			setReceiveMessage(inv[i].inv_id.amk_ac2, &inv[i].r_amk_actual_values_2, 2, TRUE);
		}
		else if (i == MOTOR_RL || i == MOTOR_RR)
		{
			setTransmitMessage(inv[i].inv_id.amk_set, &inv[i].t_amk_setpoint_1, 1, TRUE);
			setReceiveMessage(inv[i].inv_id.amk_ac1, &inv[i].r_amk_actual_values_1, 1, TRUE);
			setReceiveMessage(inv[i].inv_id.amk_ac2, &inv[i].r_amk_actual_values_2, 1, TRUE);
		}

		setTransmitMessage(inv[i].inv_id_log.amk_set, &inv[i].t_amk_setpoint_1_log, 0, FALSE);
		setTransmitMessage(inv[i].inv_id_log.amk_ac1, &inv[i].t_amk_actual_values_1_log, 0, FALSE);
		setTransmitMessage(inv[i].inv_id_log.amk_ac2, &inv[i].t_amk_actual_values_2_log, 0, FALSE);
	}

	setTransmitMessage(inv_seq.id, &inv_seq.t_inv_seq_node_1, 1, TRUE);
	setTransmitMessage(inv_seq.id, &inv_seq.t_inv_seq_node_2, 2, TRUE);

	setReceiveMessage(inv_status.id, &inv_status.r_inv_status_node_1, 1, TRUE);
	setReceiveMessage(inv_status.id, &inv_status.r_inv_status_node_2, 2, TRUE);
}

void AmkInverter_can_Run(void)
{
	for (int i = 0; i < 4; i++)
	{
		if (CanCommunication_receiveMessage(&inv[i].r_amk_actual_values_1))
		{
			inv[i].amk_actual_values_1.RecievedData[0] = inv[i].r_amk_actual_values_1.msg.data[0];
			inv[i].amk_actual_values_1.RecievedData[1] = inv[i].r_amk_actual_values_1.msg.data[1];

			CanCommunication_setMessageData(inv[i].amk_actual_values_1.RecievedData[0], inv[i].amk_actual_values_1.RecievedData[1], &inv[i].t_amk_actual_values_1_log);
			CanCommunication_transmitMessage(&inv[i].t_amk_actual_values_1_log);
		}
		if (CanCommunication_receiveMessage(&inv[i].r_amk_actual_values_2))
		{
			inv[i].amk_actual_values_2.RecievedData[0] = inv[i].r_amk_actual_values_2.msg.data[0];
			inv[i].amk_actual_values_2.RecievedData[1] = inv[i].r_amk_actual_values_2.msg.data[1];

			CanCommunication_setMessageData(inv[i].amk_actual_values_2.RecievedData[0], inv[i].amk_actual_values_2.RecievedData[1], &inv[i].t_amk_actual_values_2_log);
			CanCommunication_transmitMessage(&inv[i].t_amk_actual_values_2_log);
		}
	}

	Monitor.InverterErrorState.error_RL = inv[MOTOR_RL].amk_actual_values_1.S.AMK_bError;
	Monitor.InverterErrorState.error_FL = inv[MOTOR_FL].amk_actual_values_1.S.AMK_bError;
	Monitor.InverterErrorState.error_RR = inv[MOTOR_RR].amk_actual_values_1.S.AMK_bError;
	Monitor.InverterErrorState.error_FR = inv[MOTOR_FR].amk_actual_values_1.S.AMK_bError;

	Monitor.MotorTemp.temp_RL = inv[MOTOR_RL].amk_actual_values_2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_FL = inv[MOTOR_FL].amk_actual_values_2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_RR = inv[MOTOR_RR].amk_actual_values_2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_FR = inv[MOTOR_FR].amk_actual_values_2.S.AMK_TempMotor;

	Monitor.InverterTemp.temp_RL = inv[MOTOR_RL].amk_actual_values_2.S.AMK_TempInverter;
	Monitor.InverterTemp.temp_FL = inv[MOTOR_FL].amk_actual_values_2.S.AMK_TempInverter;
	Monitor.InverterTemp.temp_RR = inv[MOTOR_RR].amk_actual_values_2.S.AMK_TempInverter;
	Monitor.InverterTemp.temp_FR = inv[MOTOR_FR].amk_actual_values_2.S.AMK_TempInverter;

	Monitor.MotorVelocity.velocity_RL = inv[MOTOR_RL].amk_actual_values_1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_FL = inv[MOTOR_FL].amk_actual_values_1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_RR = inv[MOTOR_RR].amk_actual_values_1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_FR = inv[MOTOR_FR].amk_actual_values_1.S.AMK_ActualVelocity;

	while(IfxCpu_acquireMutex(&AmkInverterMonitorPublic.mutex))
		; // wait for the mutex
	{
		AmkInverterMonitorPublic.monitor = Monitor;
		IfxCpu_releaseMutex(&AmkInverterMonitorPublic.mutex);
	}

	if (CanCommunication_receiveMessage(&inv_status.r_inv_status_node_1))
	{
		inv_status.inv_status.RecievedData[0] = inv_status.r_inv_status_node_1.msg.data[0];
		inv_status.inv_status.RecievedData[1] = inv_status.r_inv_status_node_1.msg.data[1];
	}
	else
	{
		if (CanCommunication_receiveMessage(&inv_status.r_inv_status_node_2))
		{
			inv_status.inv_status.RecievedData[0] = inv_status.r_inv_status_node_2.msg.data[0];
			inv_status.inv_status.RecievedData[1] = inv_status.r_inv_status_node_2.msg.data[1];
		}
	}
}

void AmkInverter_Start(boolean rtd_flag)
{
	if (rtd_flag == TRUE)
	{
		for (int i = 0; i < 4; i++)
		{
//			if (inv[i].inv_on == TRUE)
//			{
//				inv[i].inv_seq_timer = 0;
//			}
//			else
//			{
//				inv[i].inv_seq_timer++;
//			}
//
//			if (inv[i].inv_seq_timer > 50)
//			{
//				seqReset(i);
//
//				inv[i].inv_seq_timer 	= 0;
//			}
//			else
//			{
//				seqSet(i);
//			}
			seqSet(i);
		}
	}
	else	// rtd_flag == FALSE
	{
		for (int i = 0; i < 4; i++)
		{
			seqReset(i);

			inv[i].inv_seq_timer = 0;
			inv[i].inv_error_reset_cnt = 0;
		}
	}
	switchWrite();

	/*Update the state for the public*/
    while(IfxCpu_acquireMutex(&AmkInverterPublic.mutex));   //Wait for the mutex
    {
//        if ((inv[MOTOR_FL].inv_on == TRUE && inv[MOTOR_FR].inv_on == TRUE) || (inv[MOTOR_RL].inv_on == TRUE && inv[MOTOR_RR].inv_on == TRUE))
//        	AmkInverterPublic.r2d	=	TRUE;
    	AmkInverterPublic.r2d	=	(inv[MOTOR_FL].inv_on || inv[MOTOR_FR].inv_on || inv[MOTOR_RL].inv_on || inv[MOTOR_RR].inv_on);
        IfxCpu_releaseMutex(&AmkInverterPublic.mutex);
    }
}

void AmkInverter_writeMessageFront(sint16 torque_left, sint16 torque_right, boolean accelerating)
{
//	if ((inv[MOTOR_FL].amk_actual_values_1.S.AMK_bError == TRUE || inv[MOTOR_FR].amk_actual_values_1.S.AMK_bError == TRUE) &&
//		(inv[MOTOR_RL].amk_actual_values_1.S.AMK_bError == FALSE && inv[MOTOR_RR].amk_actual_values_1.S.AMK_bError == FALSE))
//	{
//		invWrite(MOTOR_FL, 0, FALSE);
//		invWrite(MOTOR_FR, 0, FALSE);
//	}
//	else
//	{
//		invWrite(MOTOR_FL, torque_left, accelerating);
//		invWrite(MOTOR_FR, torque_right, accelerating);
//	}
	invWrite(MOTOR_FL, torque_left, accelerating);
	invWrite(MOTOR_FR, torque_right, accelerating);
}

void AmkInverter_writeMessageRear(sint16 torque_left, sint16 torque_right, boolean accelerating)
{
//	if ((inv[MOTOR_FL].amk_actual_values_1.S.AMK_bError == FALSE && inv[MOTOR_FR].amk_actual_values_1.S.AMK_bError == FALSE) &&
//		(inv[MOTOR_RL].amk_actual_values_1.S.AMK_bError == TRUE || inv[MOTOR_RR].amk_actual_values_1.S.AMK_bError == TRUE))
//	{
//		invWrite(MOTOR_RL, 0, FALSE);
//		invWrite(MOTOR_RR, 0, FALSE);
//	}
//	else
//	{
//		invWrite(MOTOR_RL, torque_left, accelerating);
//		invWrite(MOTOR_RR, torque_right, accelerating);
//	}
	invWrite(MOTOR_RL, torque_left, accelerating);
	invWrite(MOTOR_RR, torque_right, accelerating);
}

static void SetId(id_set_t *id, int node)
{
	id->amk_ac1 = 0x282 + node;
	id->amk_ac2 = 0x284 + node;
	id->amk_set = 0x183 + node;
}

static void setTransmitMessage(uint32 id, CanCommunication_Message *Tm, uint8 node, boolean is_standard_id)
{
	CanCommunication_Message_Config config_message_transmit;
	config_message_transmit.messageId        			= id;
	config_message_transmit.frameType        			= IfxMultican_Frame_transmit;
	config_message_transmit.dataLen          			= IfxMultican_DataLengthCode_8;
	config_message_transmit.isStandardId   				= is_standard_id;
	if (node == 0)	config_message_transmit.node 		= &CanCommunication_canNode0;
	else if (node == 1)	config_message_transmit.node 	= &CanCommunication_canNode1;
	else if (node == 2)	config_message_transmit.node 	= &CanCommunication_canNode2;
	CanCommunication_initMessage(Tm, &config_message_transmit);
}

static void setReceiveMessage(uint32 id, CanCommunication_Message *Rm, uint8 node, boolean is_standard_id)
{
	CanCommunication_Message_Config config_message_receive;
	config_message_receive.messageId        			= id;
	config_message_receive.frameType        			= IfxMultican_Frame_receive;
	config_message_receive.dataLen          			= IfxMultican_DataLengthCode_8;
	config_message_receive.isStandardId   				= is_standard_id;
	if (node == 0)	config_message_receive.node 		= &CanCommunication_canNode0;
	else if (node == 1)	config_message_receive.node 	= &CanCommunication_canNode1;
	else if (node == 2)	config_message_receive.node 	= &CanCommunication_canNode2;
	CanCommunication_initMessage(Rm, &config_message_receive);
}

static void seqSet(int i)
{
	boolean be1_on = FALSE;
	if (inv[i].inv_address == 1)		be1_on = inv_status.inv_status.S.inv1_be1_on;
	else if (inv[i].inv_address == 2)	be1_on = inv_status.inv_status.S.inv2_be1_on;
	else if (inv[i].inv_address == 5)	be1_on = inv_status.inv_status.S.inv3_be1_on;
	else if (inv[i].inv_address == 6)	be1_on = inv_status.inv_status.S.inv4_be1_on;

	boolean be2_on = FALSE;
	if (inv[i].inv_address == 1)		be2_on = inv_status.inv_status.S.inv1_be2_on;
	else if (inv[i].inv_address == 2)	be2_on = inv_status.inv_status.S.inv2_be2_on;
	else if (inv[i].inv_address == 5)	be2_on = inv_status.inv_status.S.inv3_be2_on;
	else if (inv[i].inv_address == 6)	be2_on = inv_status.inv_status.S.inv4_be2_on;

	inv[i].inv_switch.error_reset			= inv[i].amk_actual_values_1.S.AMK_bError;
	inv[i].inv_switch.dc_on 				= inv[i].amk_actual_values_1.S.AMK_bSystemReady;
	inv[i].inv_switch.torque_limit_negativ 	= 0;
	inv[i].inv_switch.torque_limit_positv 	= 0;
	inv[i].inv_switch.be1_on 				= inv[i].amk_actual_values_1.S.AMK_bQuitDcOn;
	inv[i].inv_switch.enable 				= be1_on;
	inv[i].inv_switch.inverter_on 			= be1_on;
	inv[i].inv_switch.be2_on 				= inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn;
	inv[i].inv_switch.target_velocity 		= 0;
	inv[i].inv_on 							= be2_on;

	if (inv[i].amk_actual_values_2.S.AMK_ErrorInfo == AMK_RESTART_ERROR)	inv[i].inv_error_reset_cnt	=	0;

	if (inv[i].amk_actual_values_1.S.AMK_bError == TRUE)			return;
	if (inv[i].amk_actual_values_1.S.AMK_bSystemReady == FALSE)		return;
	if (inv[i].amk_actual_values_1.S.AMK_bDcOn == FALSE)			return;
	if (inv[i].amk_actual_values_1.S.AMK_bQuitDcOn == FALSE)		return;
	if (be1_on == FALSE)											return;
	if (inv[i].amk_actual_values_1.S.AMK_bInverterOn == FALSE)		return;
	if (inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn == FALSE)	return;
	if (be2_on == FALSE)											return;
	
}

static void seqReset(int i)
{
	inv[i].inv_switch.error_reset			= FALSE;
	inv[i].inv_switch.dc_on 				= FALSE;
	inv[i].inv_switch.torque_limit_negativ 	= 0;
	inv[i].inv_switch.torque_limit_positv 	= 0;
	inv[i].inv_switch.ef_on 				= FALSE;
	inv[i].inv_switch.be1_on 				= FALSE;
	inv[i].inv_switch.enable 				= FALSE;
	inv[i].inv_switch.inverter_on 			= FALSE;
	inv[i].inv_switch.be2_on 				= FALSE;
	inv[i].inv_switch.target_velocity 		= 0;
	inv[i].inv_on 							= FALSE;
}

static void invWrite(int i, sint16 torque, boolean accelerating)
{
	// if (inv[i].inv_on == TRUE)
	// {
	// 	if (torque > 0)
	// 	{
	// 		inv[i].inv_switch.target_velocity 		= AMK_VELOCITY_LIM;
	// 		inv[i].inv_switch.torque_limit_positv 	= (torque < AMK_TORQUE_POSITIVE_LIM) ? torque : AMK_TORQUE_POSITIVE_LIM;
	// 		inv[i].inv_switch.torque_limit_negativ 	= 0;
	// 	}
	// 	else if (torque < 0)
	// 	{
	// 		inv[i].inv_switch.target_velocity 		= AMK_VELOCITY_STOP;
	// 		inv[i].inv_switch.torque_limit_positv 	= 0;
	// 		inv[i].inv_switch.torque_limit_negativ 	= (torque > AMK_TORQUE_NEGATIVE_LIM) ? torque : AMK_TORQUE_NEGATIVE_LIM;
	// 	}
	// 	else
	// 	{
	// 		if (accelerating)	inv[i].inv_switch.target_velocity	= AMK_VELOCITY_LIM;
	// 		else				inv[i].inv_switch.target_velocity	= AMK_VELOCITY_STOP;
	// 		inv[i].inv_switch.torque_limit_positv 	= 0;
	// 		inv[i].inv_switch.torque_limit_negativ 	= 0;
	// 	}
	// }
	// else
	// {
	// 	inv[i].inv_switch.target_velocity		= AMK_VELOCITY_STOP;
	// 	inv[i].inv_switch.torque_limit_positv 	= 0;
	// 	inv[i].inv_switch.torque_limit_negativ 	= 0;
	// }
	if (inv[i].inv_on == TRUE)
	{
		inv[i].inv_switch.target_velocity		= torque;
		inv[i].inv_switch.torque_limit_positv 	= AMK_TORQUE_POSITIVE_LIM;
		inv[i].inv_switch.torque_limit_negativ 	= 0;
	}
	else
	{
		inv[i].inv_switch.target_velocity		= AMK_VELOCITY_STOP;
		inv[i].inv_switch.torque_limit_positv 	= 0;
		inv[i].inv_switch.torque_limit_negativ 	= 0;
	}
	inv[i].amk_setpoint_1.S.AMK_bInverterOn 		= inv[i].inv_switch.inverter_on;
	inv[i].amk_setpoint_1.S.AMK_bDcOn 				= inv[i].inv_switch.dc_on;
	inv[i].amk_setpoint_1.S.AMK_bEnable 			= inv[i].inv_switch.enable;
	if (inv[i].inv_error_reset_cnt < MAX_AMK_ERROR_RESET)
		inv[i].amk_setpoint_1.S.AMK_bErrorReset		= inv[i].inv_switch.error_reset;
	else
		inv[i].amk_setpoint_1.S.AMK_bErrorReset		= FALSE;
	inv[i].amk_setpoint_1.S.AMK_TargetVelocity 		= inv[i].inv_switch.target_velocity;
	inv[i].amk_setpoint_1.S.AMK_TorqueLimitPositv 	= inv[i].inv_switch.torque_limit_positv;
	inv[i].amk_setpoint_1.S.AMK_TorqueLimitNegativ 	= inv[i].inv_switch.torque_limit_negativ;

	CanCommunication_setMessageData(inv[i].amk_setpoint_1.TransmitData[0], inv[i].amk_setpoint_1.TransmitData[1], &inv[i].t_amk_setpoint_1);
	CanCommunication_transmitMessage(&inv[i].t_amk_setpoint_1);

	if (inv[i].inv_switch.error_reset == TRUE && inv[i].inv_error_reset_cnt < MAX_AMK_ERROR_RESET)	inv[i].inv_error_reset_cnt++;

	CanCommunication_setMessageData(inv[i].amk_setpoint_1.TransmitData[0], inv[i].amk_setpoint_1.TransmitData[1], &inv[i].t_amk_setpoint_1_log);
	CanCommunication_transmitMessage(&inv[i].t_amk_setpoint_1_log);
}

static void switchWrite(void)
{
	for (int i = 0; i < 4; i++)
	{
		if (inv[i].inv_address == 1)
		{
			inv_seq.inv_seq.S.inv1_error 			= inv[i].amk_actual_values_1.S.AMK_bError;
			inv_seq.inv_seq.S.inv1_warn 			= inv[i].amk_actual_values_1.S.AMK_bWarn;
			inv_seq.inv_seq.S.inv1_system_ready 	= inv[i].amk_actual_values_1.S.AMK_bSystemReady;
			inv_seq.inv_seq.S.inv1_quit_dc_on 		= inv[i].amk_actual_values_1.S.AMK_bQuitDcOn;
			inv_seq.inv_seq.S.inv2_ef_on 			= inv[i].inv_switch.ef_on;
			inv_seq.inv_seq.S.inv1_be1_on 			= inv[i].inv_switch.be1_on;
			inv_seq.inv_seq.S.inv1_quit_inverter_on = inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn;
			inv_seq.inv_seq.S.inv1_be2_on 			= inv[i].inv_switch.be2_on;
		}
		else if (inv[i].inv_address == 2)
		{
			inv_seq.inv_seq.S.inv2_error 			= inv[i].amk_actual_values_1.S.AMK_bError;
			inv_seq.inv_seq.S.inv2_warn 			= inv[i].amk_actual_values_1.S.AMK_bWarn;
			inv_seq.inv_seq.S.inv2_system_ready 	= inv[i].amk_actual_values_1.S.AMK_bSystemReady;
			inv_seq.inv_seq.S.inv2_quit_dc_on 		= inv[i].amk_actual_values_1.S.AMK_bQuitDcOn;
			inv_seq.inv_seq.S.inv1_ef_on 			= inv[i].inv_switch.ef_on;
			inv_seq.inv_seq.S.inv2_be1_on 			= inv[i].inv_switch.be1_on;
			inv_seq.inv_seq.S.inv2_quit_inverter_on = inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn;
			inv_seq.inv_seq.S.inv2_be2_on 			= inv[i].inv_switch.be2_on;
		}
		else if (inv[i].inv_address == 5)
		{
			inv_seq.inv_seq.S.inv3_error 			= inv[i].amk_actual_values_1.S.AMK_bError;
			inv_seq.inv_seq.S.inv3_warn 			= inv[i].amk_actual_values_1.S.AMK_bWarn;
			inv_seq.inv_seq.S.inv3_system_ready 	= inv[i].amk_actual_values_1.S.AMK_bSystemReady;
			inv_seq.inv_seq.S.inv3_quit_dc_on 		= inv[i].amk_actual_values_1.S.AMK_bQuitDcOn;
			inv_seq.inv_seq.S.inv1_ef_on 			= inv[i].inv_switch.ef_on;
			inv_seq.inv_seq.S.inv3_be1_on 			= inv[i].inv_switch.be1_on;
			inv_seq.inv_seq.S.inv3_quit_inverter_on = inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn;
			inv_seq.inv_seq.S.inv3_be2_on 			= inv[i].inv_switch.be2_on;
		}
		else if (inv[i].inv_address == 6)
		{
			inv_seq.inv_seq.S.inv4_error 			= inv[i].amk_actual_values_1.S.AMK_bError;
			inv_seq.inv_seq.S.inv4_warn 			= inv[i].amk_actual_values_1.S.AMK_bWarn;
			inv_seq.inv_seq.S.inv4_system_ready 	= inv[i].amk_actual_values_1.S.AMK_bSystemReady;
			inv_seq.inv_seq.S.inv4_quit_dc_on 		= inv[i].amk_actual_values_1.S.AMK_bQuitDcOn;
			inv_seq.inv_seq.S.inv2_ef_on 			= inv[i].inv_switch.ef_on;
			inv_seq.inv_seq.S.inv4_be1_on 			= inv[i].inv_switch.be1_on;
			inv_seq.inv_seq.S.inv4_quit_inverter_on = inv[i].amk_actual_values_1.S.AMK_bQuitInverterOn;
			inv_seq.inv_seq.S.inv4_be2_on 			= inv[i].inv_switch.be2_on;
		}
	}

	if (inv[MOTOR_FL].inv_on == FALSE || inv[MOTOR_FR].inv_on == FALSE || inv[MOTOR_RL].inv_on == FALSE || inv[MOTOR_RR].inv_on == FALSE)
	{
		CanCommunication_setMessageData(inv_seq.inv_seq.TransmitData[0], inv_seq.inv_seq.TransmitData[1], &inv_seq.t_inv_seq_node_1);
		CanCommunication_transmitMessage(&inv_seq.t_inv_seq_node_1);
		if (inv_seq.t_inv_seq_node_1.isUpdated == FALSE)
		{
			CanCommunication_setMessageData(inv_seq.inv_seq.TransmitData[0], inv_seq.inv_seq.TransmitData[1], &inv_seq.t_inv_seq_node_2);
			CanCommunication_transmitMessage(&inv_seq.t_inv_seq_node_2);
		}
	}
}
#else
const float Inverter_peak_current = 107.2;
const float Nominal_torque = 9.8;
const uint16 InvCtr = 0x160;
boolean alreadyOn=0; 

ID_set Inverter_FL;
ID_set Inverter_RL;
ID_set Inverter_RR;
ID_set Inverter_FR;


CanCommunication_Message T_TC275_FL;
CanCommunication_Message T_TC275_RL;
CanCommunication_Message T_TC275_RR;
CanCommunication_Message T_TC275_FR;
CanCommunication_Message T_InvCtr;

CanCommunication_Message T_INV_FL_AMK_Actual_Values1_log;
CanCommunication_Message T_INV_RL_AMK_Actual_Values1_log;
CanCommunication_Message T_INV_RR_AMK_Actual_Values1_log;
CanCommunication_Message T_INV_FR_AMK_Actual_Values1_log;
CanCommunication_Message T_INV_FL_AMK_Actual_Values2_log;
CanCommunication_Message T_INV_RL_AMK_Actual_Values2_log;
CanCommunication_Message T_INV_RR_AMK_Actual_Values2_log;
CanCommunication_Message T_INV_FR_AMK_Actual_Values2_log;

CanCommunication_Message R_Inverter_FL_1;
CanCommunication_Message R_Inverter_RL_1;
CanCommunication_Message R_Inverter_RR_1;
CanCommunication_Message R_Inverter_FR_1;
CanCommunication_Message R_Inverter_FL_2;
CanCommunication_Message R_Inverter_RL_2;
CanCommunication_Message R_Inverter_RR_2;
CanCommunication_Message R_Inverter_FR_2;


amkActualValues1 INV_FL_AMK_Actual_Values1;
amkActualValues1 INV_RL_AMK_Actual_Values1;
amkActualValues1 INV_RR_AMK_Actual_Values1;
amkActualValues1 INV_FR_AMK_Actual_Values1;

amkActualValues2 INV_FL_AMK_Actual_Values2;
amkActualValues2 INV_RL_AMK_Actual_Values2;
amkActualValues2 INV_RR_AMK_Actual_Values2;
amkActualValues2 INV_FR_AMK_Actual_Values2;

AmkInverterPublic_t AmkInverterPublic;

amkSetpoint1 INV_FL_AMK_Setpoint1;
amkSetpoint1 INV_RL_AMK_Setpoint1;
amkSetpoint1 INV_RR_AMK_Setpoint1;
amkSetpoint1 INV_FR_AMK_Setpoint1;
Inv_switch_msg_t Inv_switch_msg;

boolean AmkInverterError = FALSE;

AmkActualValues1_log_t INV_FL_AMK_Actual_Values1_log;
AmkActualValues1_log_t INV_RL_AMK_Actual_Values1_log;
AmkActualValues1_log_t INV_RR_AMK_Actual_Values1_log;
AmkActualValues1_log_t INV_FR_AMK_Actual_Values1_log;

AmkActualValues2_log_t INV_FL_AMK_Actual_Values2_log;
AmkActualValues2_log_t INV_RL_AMK_Actual_Values2_log;
AmkActualValues2_log_t INV_RR_AMK_Actual_Values2_log;
AmkActualValues2_log_t INV_FR_AMK_Actual_Values2_log;

void AmkInverter_can_init(void);
void AmkInverter_can_Run(void);
void AmkInverter_can_write(amkSetpoint1 *INV, CanCommunication_Message TC, uint16 tV);

static void setPointInit(amkSetpoint1 *setpoint);

static void setReceiveMessage(uint16_t ID, CanCommunication_Message *Rm,uint8 node);
static void setTransmitMessage(uint16_t ID, CanCommunication_Message *Tm,uint8 node);
void AmkInverter_writeMessage(uint16 Value1, uint16 Value2);
void AmkInverter_writeMessage2(uint16 Value1, uint16 Value2);

struct setSwitch
{
    uint8 DCon;
    uint8 Enable;
    uint8 inverter;
    uint16 posTorquelimit;
    int16_t negTorquelimit;
    uint8 ErrorReset;
    uint32 Checker;
    boolean BE1;
    boolean BE2;
    boolean EF;
};

// struct Monitor
// {
//     int InverterTemp;
//     struct {
//         uint16 error_FL;
//         uint16 error_RL;
//         uint16 error_RR;
//         uint16 error_FR;
//     }InverterErrorState;
//     struct {
//         uint16 temp_FL;
//         uint16 temp_RL;
//         uint16 temp_RR;
//         uint16 temp_FR;
//     }MotorTemp;
//     struct{
//         uint16 velocity_FL;
//         uint16 velocity_RL;
//         uint16 velocity_RR;
//         uint16 velocity_FR;
//     } MotorVelocity;
//     // struct MotorCurrent{
//     //     uint16 velocity_RL;
//     //     uint16 velocity_FL;
//     //     uint16 velocity_RR;
//     //     uint16 velocity_FR;
//     // }
// };

uint32 AmkState_S2cnt = 0;
uint32 AmkState_S3cnt = 0;

const uint32 AmkState_constS2threshold = 100;
const uint32 AmkState_constS3threshold = 100;

struct Monitor Monitor;//
struct setSwitch SWITCH = {0,0,0,0,0,0};
AmkState_t AmkState = AmkState_S0;

AmkInverterMonitorPublic_t AmkInverterMonitorPublic;

void SET_ID(ID_set *IN, int node)
{
	IN->ID_AMK_Ac1 = 0x282 + node;
	IN->ID_AMK_Ac2 = 0x284 + node;
	IN->ID_AMK_Set = 0x183 + node;
}

void AmkInverter_can_init(void)
{   
    AmkState = AmkState_S0;
    
    //Previous Front/Rear Split setting
	SET_ID(&Inverter_FL,5); //2, FR, node1
	SET_ID(&Inverter_RL,6); //4, FL, node2
	SET_ID(&Inverter_RR,1); //3, RL, node2
	SET_ID(&Inverter_FR,2); //1, RR, node1

    //New Right/Left Split setting
    
	SET_ID(&Inverter_FR,5); //Inverter #2,Node 1, Address 5
	SET_ID(&Inverter_RR,2); //Inverter #1,Node 1, Address 2

    SET_ID(&Inverter_FL,6); //Inverter #4, Node 2, Address 6
	SET_ID(&Inverter_RL,1); //Inverter #3, Node 2, Address 1

    /**************************************Transmit***************************************************/
    setTransmitMessage(Inverter_FR.ID_AMK_Set, &T_TC275_FR,1);
    setTransmitMessage(Inverter_RR.ID_AMK_Set, &T_TC275_RR,1);
    
    setTransmitMessage(Inverter_FL.ID_AMK_Set, &T_TC275_FL,2);
    setTransmitMessage(Inverter_RL.ID_AMK_Set, &T_TC275_RL,2);
    
    setTransmitMessage(InvCtr,&T_InvCtr,1);

    /**************************************Receive***************************************************/
    setReceiveMessage(Inverter_FR.ID_AMK_Ac1, &R_Inverter_FR_1,1);
    setReceiveMessage(Inverter_FR.ID_AMK_Ac2, &R_Inverter_FR_2,1);
    
    setReceiveMessage(Inverter_RR.ID_AMK_Ac1, &R_Inverter_RR_1,1);
    setReceiveMessage(Inverter_RR.ID_AMK_Ac2, &R_Inverter_RR_2,1);

    
    setReceiveMessage(Inverter_FL.ID_AMK_Ac1, &R_Inverter_FL_1,2);
    setReceiveMessage(Inverter_FL.ID_AMK_Ac2, &R_Inverter_FL_2,2);

    setReceiveMessage(Inverter_RL.ID_AMK_Ac1, &R_Inverter_RL_1,2);
    setReceiveMessage(Inverter_RL.ID_AMK_Ac2, &R_Inverter_RL_2,2);

    
    // {
    //     CanCommunication_Message_Config config_Message8_Recive;
    //     config_Message8_Recive.messageId        =   STM32ID;
    //     config_Message8_Recive.frameType        =   IfxMultican_Frame_receive;
    //     config_Message8_Recive.dataLen          =   IfxMultican_DataLengthCode_8;
    //     config_Message8_Recive.node             =   &CanCommunication_canNode1;
    //     CanCommunication_initMessage(&STM32A, &config_Message8_Recive);
    // }
    /**************************************Initial setpoint***************************************************/ 
    setPointInit(&INV_FL_AMK_Setpoint1);
    setPointInit(&INV_RL_AMK_Setpoint1);
    setPointInit(&INV_RR_AMK_Setpoint1);
    setPointInit(&INV_FR_AMK_Setpoint1);

}

void AmkInverter_can_Run(void)
{
    if(CanCommunication_receiveMessage(&R_Inverter_FL_1))
    {
    	INV_FL_AMK_Actual_Values1.RecievedData[0]      =   R_Inverter_FL_1.msg.data[0];
    	INV_FL_AMK_Actual_Values1.RecievedData[1]      =   R_Inverter_FL_1.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_FL_2))
    {
        INV_FL_AMK_Actual_Values2.RecievedData[0]      =   R_Inverter_FL_2.msg.data[0];
        INV_FL_AMK_Actual_Values2.RecievedData[1]      =   R_Inverter_FL_2.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_RL_1))
    {
        INV_RL_AMK_Actual_Values1.RecievedData[0]      =   R_Inverter_RL_1.msg.data[0];
        INV_RL_AMK_Actual_Values1.RecievedData[1]      =   R_Inverter_RL_1.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_RL_2))
    {
        INV_RL_AMK_Actual_Values2.RecievedData[0]      =   R_Inverter_RL_2.msg.data[0];
        INV_RL_AMK_Actual_Values2.RecievedData[1]      =   R_Inverter_RL_2.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_RR_1))
    {
    	INV_RR_AMK_Actual_Values1.RecievedData[0]      =   R_Inverter_RR_1.msg.data[0];
    	INV_RR_AMK_Actual_Values1.RecievedData[1]      =   R_Inverter_RR_1.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_RR_2))
    {
        INV_RR_AMK_Actual_Values2.RecievedData[0]      =   R_Inverter_RR_2.msg.data[0];
        INV_RR_AMK_Actual_Values2.RecievedData[1]      =   R_Inverter_RR_2.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_FR_1))
    {
        INV_FR_AMK_Actual_Values1.RecievedData[0]      =   R_Inverter_FR_1.msg.data[0];
        INV_FR_AMK_Actual_Values1.RecievedData[1]      =   R_Inverter_FR_1.msg.data[1];
    }
    if(CanCommunication_receiveMessage(&R_Inverter_FR_2))
    {
        INV_FR_AMK_Actual_Values2.RecievedData[0]      =   R_Inverter_FR_2.msg.data[0];
        INV_FR_AMK_Actual_Values2.RecievedData[1]      =   R_Inverter_FR_2.msg.data[1];
    }

	Monitor.InverterErrorState.error_RL = INV_RL_AMK_Actual_Values1.S.AMK_bSError;
	Monitor.InverterErrorState.error_FL = INV_FL_AMK_Actual_Values1.S.AMK_bSError;
	Monitor.InverterErrorState.error_RR = INV_RR_AMK_Actual_Values1.S.AMK_bSError;
	Monitor.InverterErrorState.error_FR = INV_FR_AMK_Actual_Values1.S.AMK_bSError;

	// Monitor.MotorTemp.temp_RL = INV_RL_AMK_Actual_Values2.S.AMK_TempMotor * 0.1;
	// Monitor.MotorTemp.temp_FL = INV_FL_AMK_Actual_Values2.S.AMK_TempMotor * 0.1;
	// Monitor.MotorTemp.temp_RR = INV_RR_AMK_Actual_Values2.S.AMK_TempMotor * 0.1;
	// Monitor.MotorTemp.temp_FR = INV_FR_AMK_Actual_Values2.S.AMK_TempMotor * 0.1;

	Monitor.MotorTemp.temp_RL = INV_RL_AMK_Actual_Values2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_FL = INV_FL_AMK_Actual_Values2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_RR = INV_RR_AMK_Actual_Values2.S.AMK_TempMotor;
	Monitor.MotorTemp.temp_FR = INV_FR_AMK_Actual_Values2.S.AMK_TempMotor;

	Monitor.MotorVelocity.velocity_RL = INV_RL_AMK_Actual_Values1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_FL = INV_FL_AMK_Actual_Values1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_RR = INV_RR_AMK_Actual_Values1.S.AMK_ActualVelocity;
	Monitor.MotorVelocity.velocity_FR = INV_FR_AMK_Actual_Values1.S.AMK_ActualVelocity;
	Monitor.InverterTemp = INV_RL_AMK_Actual_Values2.S.AMK_TempInverter;
	SWITCH.Checker += 1;

	while(IfxCpu_acquireMutex(&AmkInverterMonitorPublic.mutex))
		; // wait for the mutex
	{
		AmkInverterMonitorPublic.monitor = Monitor;
		IfxCpu_releaseMutex(&AmkInverterMonitorPublic.mutex);
	}
}

void AmkInverter_can_write(amkSetpoint1 *INV, CanCommunication_Message TC, uint16 tV)
{    
    if (SWITCH.DCon&&SWITCH.Enable&&SWITCH.inverter&&(SWITCH.posTorquelimit>0))INV->S.AMK_Torque_setpoint = tV;
    INV->S.AMK_bDcOn = SWITCH.DCon;
    INV->S.AMK_bEnable = SWITCH.Enable;
    INV->S.AMK_bInverterOn = SWITCH.inverter;
    INV->S.AMK_TorqueLimitPositv  = SWITCH.posTorquelimit;
    INV->S.AMK_TorqueLimitNegativ = SWITCH.negTorquelimit;
    // if (SWITCH.ErrorReset){
        INV->S.AMK_bErrorReset = SWITCH.ErrorReset;
        // SWITCH.ErrorReset = 0;
    // }
    // else{
        // INV->S.AMK_bErrorReset = 0;
    // }
    CanCommunication_setMessageData(INV->TransmitData[0],INV->TransmitData[1], &TC);

    CanCommunication_transmitMessage(&TC);

}

void InverterControlSet(){
    Inv_switch_msg.B.EFon = SWITCH.EF;
    Inv_switch_msg.B.BE1on = SWITCH.BE1;
    Inv_switch_msg.B.BE2on = SWITCH.BE2;
    Inv_switch_msg.B.Remain = 0x1231;
    CanCommunication_setMessageData(Inv_switch_msg.TransmitData[0],Inv_switch_msg.TransmitData[1], &T_InvCtr);

    CanCommunication_transmitMessage(&T_InvCtr);
}

void AmkInverter_writeMessage(uint16 Value1, uint16 Value2)
{

    AmkInverter_can_write(&INV_FL_AMK_Setpoint1,T_TC275_FL,Value1);
    AmkInverter_can_write(&INV_FR_AMK_Setpoint1,T_TC275_FR,Value2);
    // if (Inv_switch_msg.B.BE1on||Inv_switch_msg.B.BE2on||Inv_switch_msg.B.EFon){
    if(alreadyOn != 0){
        InverterControlSet();
    }    
    // }

}
void AmkInverter_writeMessage2(uint16 Value1, uint16 Value2)
{    

    AmkInverter_can_write(&INV_RR_AMK_Setpoint1,T_TC275_RR,Value1);
    AmkInverter_can_write(&INV_RL_AMK_Setpoint1,T_TC275_RL,Value2);
}

void AmkInverter_Start(boolean rtdFlag)
{
	if(AmkState == AmkState_S0)
	{
		SWITCH.DCon = 0;
		SWITCH.negTorquelimit = 0;
		SWITCH.posTorquelimit = 0;
		SWITCH.EF = 0;
		SWITCH.BE1 = 0;
		SWITCH.BE2 = 0;
		SWITCH.Enable = 0;
		SWITCH.inverter = 0;
	}

	/*Inverter Start Sequence*/
	if(rtdFlag == TRUE)
	{
		/*Inverter Error Check*/
		if(INV_FL_AMK_Actual_Values1.S.AMK_bSError | INV_FR_AMK_Actual_Values1.S.AMK_bSError |
		    INV_RL_AMK_Actual_Values1.S.AMK_bSError | INV_RR_AMK_Actual_Values1.S.AMK_bSError)
		{
			AmkInverterError = TRUE;
		}
		else
		{
			AmkInverterError = FALSE;
		}

		/*State 0: Power On*/
		if(AmkState == AmkState_S0)
		{

			/*Try to reset the errors and return to S0*/
			// if(alreadyOn == 0 && AmkInverterError == TRUE)
			if(AmkInverterError == TRUE)
			{
				SWITCH.ErrorReset = TRUE;
				AmkState = AmkState_S0;
			}

			/*When the errors are cleared -> To the state S1*/
			if((INV_FL_AMK_Actual_Values1.S.AMK_bSystemReady & INV_FR_AMK_Actual_Values1.S.AMK_bSystemReady &
			       INV_RL_AMK_Actual_Values1.S.AMK_bSystemReady & INV_RR_AMK_Actual_Values1.S.AMK_bSystemReady))
			{
				SWITCH.ErrorReset = FALSE;
				AmkState = AmkState_S1;
			}
		}
		/*State 1: System Ready*/
		else if(AmkState == AmkState_S1)
		{
			if(alreadyOn == 0)
			{
				if(!(INV_FL_AMK_Actual_Values1.S.AMK_bSystemReady & INV_FR_AMK_Actual_Values1.S.AMK_bSystemReady &
				       INV_RL_AMK_Actual_Values1.S.AMK_bSystemReady & INV_RR_AMK_Actual_Values1.S.AMK_bSystemReady))
				{
					AmkState = AmkState_S0;
				}

				SWITCH.EF = 0;
				SWITCH.BE1 = 0;
				SWITCH.BE2 = 0;
				SWITCH.Enable = 0;
				SWITCH.inverter = 0;
				SWITCH.negTorquelimit = 0;
				SWITCH.posTorquelimit = 0;

				SWITCH.DCon = 1;
				if(INV_FL_AMK_Actual_Values1.S.AMK_bDcOn & INV_FR_AMK_Actual_Values1.S.AMK_bDcOn &
				    INV_RL_AMK_Actual_Values1.S.AMK_bDcOn & INV_RR_AMK_Actual_Values1.S.AMK_bDcOn)
				{
					AmkState = AmkState_S2;
				}
			}
		}
		/*State 2: DC On*/
		else if(AmkState == AmkState_S2)
		{
			// TODO: Error state
			alreadyOn = TRUE;
			SWITCH.EF = 1;
			SWITCH.BE1 = 0;
			SWITCH.Enable = 0;
			SWITCH.inverter = 0;
			AmkState_S2cnt++;
			if(AmkState_S2cnt > AmkState_constS2threshold)
			{
				AmkState = AmkState_S3;
			}
			else
			{
				AmkState = AmkState_S2;
			}
		}
		/*State 3: EF on*/
		else if(AmkState == AmkState_S3)
		{
			// TODO: Error state
			alreadyOn = TRUE;
			SWITCH.EF = 1;
			SWITCH.BE1 = 1;
			SWITCH.Enable = 0;
			SWITCH.inverter = 0;
			AmkState_S3cnt++;
			if(AmkState_S3cnt > AmkState_constS3threshold)
			{
				AmkState = AmkState_S4;
			}
			else
			{
				AmkState = AmkState_S3;
			}
		}
		/*State 4: BE1 on*/
		else if(AmkState == AmkState_S4)
		{
			// TODO: Error state
			alreadyOn = TRUE;
			SWITCH.EF = 1;
			SWITCH.BE1 = 1;
			SWITCH.Enable = 1;
			SWITCH.inverter = 1;
			if(INV_FL_AMK_Actual_Values1.S.AMK_bInverterOn && INV_FR_AMK_Actual_Values1.S.AMK_bInverterOn &&
			    INV_RL_AMK_Actual_Values1.S.AMK_bInverterOn && INV_RR_AMK_Actual_Values1.S.AMK_bInverterOn &&
			    INV_FL_AMK_Actual_Values1.S.AMK_bQuitInverterOn && INV_FR_AMK_Actual_Values1.S.AMK_bQuitInverterOn &&
			    INV_RL_AMK_Actual_Values1.S.AMK_bQuitInverterOn && INV_RR_AMK_Actual_Values1.S.AMK_bQuitInverterOn)
			{
				AmkState = AmkState_S5;
			}
			else
			{
				AmkState = AmkState_S4;
			}
		}
		/*State 5: Enable/InverterEnable on*/
		else if(AmkState == AmkState_S5)
		{
			// TODO: Error state
			alreadyOn = TRUE;
			SWITCH.EF = 1;
			SWITCH.BE1 = 1;
			SWITCH.Enable = 1;
			SWITCH.inverter = 1;

			/*Check again*/
			if(INV_FL_AMK_Actual_Values1.S.AMK_bInverterOn && INV_FR_AMK_Actual_Values1.S.AMK_bInverterOn &&
			    INV_RL_AMK_Actual_Values1.S.AMK_bInverterOn && INV_RR_AMK_Actual_Values1.S.AMK_bInverterOn &&
			    INV_FL_AMK_Actual_Values1.S.AMK_bQuitInverterOn && INV_FR_AMK_Actual_Values1.S.AMK_bQuitInverterOn &&
			    INV_RL_AMK_Actual_Values1.S.AMK_bQuitInverterOn && INV_RR_AMK_Actual_Values1.S.AMK_bQuitInverterOn)
			{
				SWITCH.BE2 = 1;
				SWITCH.posTorquelimit = AMK_TORQUE_LIM;
				SWITCH.negTorquelimit = -AMK_TORQUE_LIM;
				AmkState = AmkState_RTD;
			}
			else
			{
				AmkState = AmkState_S5;
			}
		}
	}
    else //rtdFlag is FALSE
    {
        SWITCH.DCon = 0;
		SWITCH.negTorquelimit = 0;
		SWITCH.posTorquelimit = 0;
		SWITCH.EF = 0;
		SWITCH.BE1 = 0;
		SWITCH.BE2 = 0;
		SWITCH.Enable = 0;
		SWITCH.inverter = 0;
        alreadyOn = FALSE;
        AmkState = AmkState_S0;
        return;
    }

    /*Update the state for the public*/
    while(IfxCpu_acquireMutex(&AmkInverterPublic.mutex));   //Wait for the mutex
    {
        AmkInverterPublic.r2d = AmkState;
        IfxCpu_releaseMutex(&AmkInverterPublic.mutex);
    }
}

static void setPointInit(amkSetpoint1 *setpoint){
    setpoint->S.AMK_bInverterOn = FALSE;
    setpoint->S.AMK_bDcOn = FALSE;
    setpoint->S.AMK_bEnable = FALSE;
    setpoint->S.AMK_bErrorReset = FALSE;
    setpoint->S.AMK_TorqueLimitPositv = 0;
    setpoint->S.AMK_TorqueLimitNegativ = 0;
}

static void setReceiveMessage(uint16_t ID, CanCommunication_Message *Rm,uint8 node){

    CanCommunication_Message_Config config_Message_Recive;
    config_Message_Recive.messageId        =   ID;
    config_Message_Recive.frameType        =   IfxMultican_Frame_receive;
    config_Message_Recive.dataLen          =   IfxMultican_DataLengthCode_8;
    config_Message_Recive.isStandardId     =   TRUE;
    if (node == 0){
        config_Message_Recive.node             =   &CanCommunication_canNode0;
    }
    else if (node == 1){
        config_Message_Recive.node             =   &CanCommunication_canNode1;
    }
    else{
        config_Message_Recive.node             =   &CanCommunication_canNode2;
    }

    CanCommunication_initMessage(Rm, &config_Message_Recive);

}

static void setTransmitMessage(uint16_t ID, CanCommunication_Message *Tm,uint8 node){

    CanCommunication_Message_Config config_Message_Transmit;
    config_Message_Transmit.messageId        =   ID;
    config_Message_Transmit.frameType        =   IfxMultican_Frame_transmit;
    config_Message_Transmit.dataLen          =   IfxMultican_DataLengthCode_8;
    config_Message_Transmit.isStandardId       =   TRUE;
    if (node == 0){
        config_Message_Transmit.node             =   &CanCommunication_canNode0;
    }
    else if (node == 1){
        config_Message_Transmit.node             =   &CanCommunication_canNode1;
    }
    else{
        config_Message_Transmit.node             =   &CanCommunication_canNode2;
    }

    CanCommunication_initMessage(Tm, &config_Message_Transmit);

}

void AMKInverter_initLoggingMessage(void) {

	//Init CAN msg of AMK_Actual_Values1
	setTransmitMessage(Inverter_FL.ID_AMK_Ac1 + 0x275000, &T_INV_FL_AMK_Actual_Values1_log,0);
	setTransmitMessage(Inverter_RL.ID_AMK_Ac1 + 0x275000, &T_INV_RL_AMK_Actual_Values1_log,0);
	setTransmitMessage(Inverter_FR.ID_AMK_Ac1 + 0x275000, &T_INV_FR_AMK_Actual_Values1_log,0);
	setTransmitMessage(Inverter_RR.ID_AMK_Ac1 + 0x275000, &T_INV_RR_AMK_Actual_Values1_log,0);
	//Init CAN msg of AMK_Actual_Values1
	setTransmitMessage(Inverter_FL.ID_AMK_Ac2 + 0x275000, &T_INV_FL_AMK_Actual_Values2_log,0);
	setTransmitMessage(Inverter_RL.ID_AMK_Ac2 + 0x275000, &T_INV_RL_AMK_Actual_Values2_log,0);
    setTransmitMessage(Inverter_RR.ID_AMK_Ac2 + 0x275000, &T_INV_RR_AMK_Actual_Values2_log,0);
	setTransmitMessage(Inverter_FR.ID_AMK_Ac2 + 0x275000, &T_INV_FR_AMK_Actual_Values2_log,0);

}

void AMKInverter_runLogging(void) {

	//Log AMK_Actual_Values1
	INV_FL_AMK_Actual_Values1_log.S.AMK_bReserve = INV_FL_AMK_Actual_Values1.S.AMK_bReserve;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bSystemReady = INV_FL_AMK_Actual_Values1.S.AMK_bSystemReady;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bSError = INV_FL_AMK_Actual_Values1.S.AMK_bSError;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bWarn = INV_FL_AMK_Actual_Values1.S.AMK_bWarn;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bQuitDcOn = INV_FL_AMK_Actual_Values1.S.AMK_bQuitDcOn;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bDcOn = INV_FL_AMK_Actual_Values1.S.AMK_bDcOn;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bQuitInverterOn = INV_FL_AMK_Actual_Values1.S.AMK_bQuitInverterOn;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bInverterOn = INV_FL_AMK_Actual_Values1.S.AMK_bInverterOn;
	INV_FL_AMK_Actual_Values1_log.S.AMK_bDerating = INV_FL_AMK_Actual_Values1.S.AMK_bDerating;
	INV_FL_AMK_Actual_Values1_log.S.AMK_ActualVelocity = INV_FL_AMK_Actual_Values1.S.AMK_ActualVelocity;
	INV_FL_AMK_Actual_Values1_log.S.AMK_TorqueCurrent = INV_FL_AMK_Actual_Values1.S.AMK_TorqueCurrent;
	INV_FL_AMK_Actual_Values1_log.S.AMK_MagnetizingCurrent = INV_FL_AMK_Actual_Values1.S.AMK_MagnetizingCurrent;

	INV_RL_AMK_Actual_Values1_log.S.AMK_bReserve = INV_RL_AMK_Actual_Values1.S.AMK_bReserve;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bSystemReady = INV_RL_AMK_Actual_Values1.S.AMK_bSystemReady;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bSError = INV_RL_AMK_Actual_Values1.S.AMK_bSError;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bWarn = INV_RL_AMK_Actual_Values1.S.AMK_bWarn;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bQuitDcOn = INV_RL_AMK_Actual_Values1.S.AMK_bQuitDcOn;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bDcOn = INV_RL_AMK_Actual_Values1.S.AMK_bDcOn;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bQuitInverterOn = INV_RL_AMK_Actual_Values1.S.AMK_bQuitInverterOn;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bInverterOn = INV_RL_AMK_Actual_Values1.S.AMK_bInverterOn;
	INV_RL_AMK_Actual_Values1_log.S.AMK_bDerating = INV_RL_AMK_Actual_Values1.S.AMK_bDerating;
	INV_RL_AMK_Actual_Values1_log.S.AMK_ActualVelocity = INV_RL_AMK_Actual_Values1.S.AMK_ActualVelocity;
	INV_RL_AMK_Actual_Values1_log.S.AMK_TorqueCurrent = INV_RL_AMK_Actual_Values1.S.AMK_TorqueCurrent;
	INV_RL_AMK_Actual_Values1_log.S.AMK_MagnetizingCurrent = INV_RL_AMK_Actual_Values1.S.AMK_MagnetizingCurrent;

	INV_RR_AMK_Actual_Values1_log.S.AMK_bReserve = INV_RR_AMK_Actual_Values1.S.AMK_bReserve;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bSystemReady = INV_RR_AMK_Actual_Values1.S.AMK_bSystemReady;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bSError = INV_RR_AMK_Actual_Values1.S.AMK_bSError;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bWarn = INV_RR_AMK_Actual_Values1.S.AMK_bWarn;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bQuitDcOn = INV_RR_AMK_Actual_Values1.S.AMK_bQuitDcOn;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bDcOn = INV_RR_AMK_Actual_Values1.S.AMK_bDcOn;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bQuitInverterOn = INV_RR_AMK_Actual_Values1.S.AMK_bQuitInverterOn;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bInverterOn = INV_RR_AMK_Actual_Values1.S.AMK_bInverterOn;
	INV_RR_AMK_Actual_Values1_log.S.AMK_bDerating = INV_RR_AMK_Actual_Values1.S.AMK_bDerating;
	INV_RR_AMK_Actual_Values1_log.S.AMK_ActualVelocity = INV_RR_AMK_Actual_Values1.S.AMK_ActualVelocity;
	INV_RR_AMK_Actual_Values1_log.S.AMK_TorqueCurrent = INV_RR_AMK_Actual_Values1.S.AMK_TorqueCurrent;
	INV_RR_AMK_Actual_Values1_log.S.AMK_MagnetizingCurrent = INV_RR_AMK_Actual_Values1.S.AMK_MagnetizingCurrent;

	INV_FR_AMK_Actual_Values1_log.S.AMK_bReserve = INV_FR_AMK_Actual_Values1.S.AMK_bReserve;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bSystemReady = INV_FR_AMK_Actual_Values1.S.AMK_bSystemReady;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bSError = INV_FR_AMK_Actual_Values1.S.AMK_bSError;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bWarn = INV_FR_AMK_Actual_Values1.S.AMK_bWarn;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bQuitDcOn = INV_FR_AMK_Actual_Values1.S.AMK_bQuitDcOn;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bDcOn = INV_FR_AMK_Actual_Values1.S.AMK_bDcOn;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bQuitInverterOn = INV_FR_AMK_Actual_Values1.S.AMK_bQuitInverterOn;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bInverterOn = INV_FR_AMK_Actual_Values1.S.AMK_bInverterOn;
	INV_FR_AMK_Actual_Values1_log.S.AMK_bDerating = INV_FR_AMK_Actual_Values1.S.AMK_bDerating;
	INV_FR_AMK_Actual_Values1_log.S.AMK_ActualVelocity = INV_FR_AMK_Actual_Values1.S.AMK_ActualVelocity;
	INV_FR_AMK_Actual_Values1_log.S.AMK_TorqueCurrent = INV_FR_AMK_Actual_Values1.S.AMK_TorqueCurrent;
	INV_FR_AMK_Actual_Values1_log.S.AMK_MagnetizingCurrent = INV_FR_AMK_Actual_Values1.S.AMK_MagnetizingCurrent;
	//Log AMK_Actual_Values2
	INV_FL_AMK_Actual_Values2_log.S.AMK_TempMotor = INV_FL_AMK_Actual_Values2.S.AMK_TempMotor;
	INV_FL_AMK_Actual_Values2_log.S.AMK_TempInverter = INV_FL_AMK_Actual_Values2.S.AMK_TempInverter;
	INV_FL_AMK_Actual_Values2_log.S.AMK_ErrorInfo = INV_FL_AMK_Actual_Values2.S.AMK_ErrorInfo;
	INV_FL_AMK_Actual_Values2_log.S.AMK_TempIGBT = INV_FL_AMK_Actual_Values2.S.AMK_TempIGBT;

	INV_RL_AMK_Actual_Values2_log.S.AMK_TempMotor = INV_RL_AMK_Actual_Values2.S.AMK_TempMotor;
	INV_RL_AMK_Actual_Values2_log.S.AMK_TempInverter = INV_RL_AMK_Actual_Values2.S.AMK_TempInverter;
	INV_RL_AMK_Actual_Values2_log.S.AMK_ErrorInfo = INV_RL_AMK_Actual_Values2.S.AMK_ErrorInfo;
	INV_RL_AMK_Actual_Values2_log.S.AMK_TempIGBT = INV_RL_AMK_Actual_Values2.S.AMK_TempIGBT;

	INV_RR_AMK_Actual_Values2_log.S.AMK_TempMotor = INV_RR_AMK_Actual_Values2.S.AMK_TempMotor;
	INV_RR_AMK_Actual_Values2_log.S.AMK_TempInverter = INV_RR_AMK_Actual_Values2.S.AMK_TempInverter;
	INV_RR_AMK_Actual_Values2_log.S.AMK_ErrorInfo = INV_RR_AMK_Actual_Values2.S.AMK_ErrorInfo;
	INV_RR_AMK_Actual_Values2_log.S.AMK_TempIGBT = INV_RR_AMK_Actual_Values2.S.AMK_TempIGBT;

	INV_FR_AMK_Actual_Values2_log.S.AMK_TempMotor = INV_FR_AMK_Actual_Values2.S.AMK_TempMotor;
	INV_FR_AMK_Actual_Values2_log.S.AMK_TempInverter = INV_FR_AMK_Actual_Values2.S.AMK_TempInverter;
	INV_FR_AMK_Actual_Values2_log.S.AMK_ErrorInfo = INV_FR_AMK_Actual_Values2.S.AMK_ErrorInfo;
	INV_FR_AMK_Actual_Values2_log.S.AMK_TempIGBT = INV_FR_AMK_Actual_Values2.S.AMK_TempIGBT;

	//Set CAN msg of AMK_Actual_Values1
	CanCommunication_setMessageData(INV_FL_AMK_Actual_Values1_log.TransmitData[0], INV_FL_AMK_Actual_Values1_log.TransmitData[1], &T_INV_FL_AMK_Actual_Values1_log);
	CanCommunication_setMessageData(INV_RL_AMK_Actual_Values1_log.TransmitData[0], INV_RL_AMK_Actual_Values1_log.TransmitData[1], &T_INV_RL_AMK_Actual_Values1_log);
	CanCommunication_setMessageData(INV_RR_AMK_Actual_Values1_log.TransmitData[0], INV_RR_AMK_Actual_Values1_log.TransmitData[1], &T_INV_RR_AMK_Actual_Values1_log);
	CanCommunication_setMessageData(INV_FR_AMK_Actual_Values1_log.TransmitData[0], INV_FR_AMK_Actual_Values1_log.TransmitData[1], &T_INV_FR_AMK_Actual_Values1_log);
	//Set CAN msg of AMK_Actual_Values2
	CanCommunication_setMessageData(INV_FL_AMK_Actual_Values2_log.TransmitData[0], INV_FL_AMK_Actual_Values2_log.TransmitData[1], &T_INV_FL_AMK_Actual_Values2_log);
	CanCommunication_setMessageData(INV_RL_AMK_Actual_Values2_log.TransmitData[0], INV_RL_AMK_Actual_Values2_log.TransmitData[1], &T_INV_RL_AMK_Actual_Values2_log);
	CanCommunication_setMessageData(INV_RR_AMK_Actual_Values2_log.TransmitData[0], INV_RR_AMK_Actual_Values2_log.TransmitData[1], &T_INV_RR_AMK_Actual_Values2_log);
	CanCommunication_setMessageData(INV_FR_AMK_Actual_Values2_log.TransmitData[0], INV_FR_AMK_Actual_Values2_log.TransmitData[1], &T_INV_FR_AMK_Actual_Values2_log);

	//Transmit CAM msg of AMK_Actual_Values1 to node0(main bus)
	CanCommunication_transmitMessage(&T_INV_FL_AMK_Actual_Values1_log);
	CanCommunication_transmitMessage(&T_INV_RL_AMK_Actual_Values1_log);
	CanCommunication_transmitMessage(&T_INV_RR_AMK_Actual_Values1_log);
	CanCommunication_transmitMessage(&T_INV_FR_AMK_Actual_Values1_log);
	//Transmit CAM msg of AMK_Actual_Values2 to node0(main bus)
	CanCommunication_transmitMessage(&T_INV_FL_AMK_Actual_Values2_log);
	CanCommunication_transmitMessage(&T_INV_RL_AMK_Actual_Values2_log);
	CanCommunication_transmitMessage(&T_INV_RR_AMK_Actual_Values2_log);
	CanCommunication_transmitMessage(&T_INV_FR_AMK_Actual_Values2_log);

}
#endif
