#ifndef AMKINVERTER_CAN_H_
#define AMKINVERTER_CAN_H_

#include "CanCommunication.h"
#include "Configuration.h"
#include "ConfigurationIsr.h"
#include "Multican.h"
#include <Ifx_Types.h>
#include <stdint.h>

#define AMK_MODE 0

#if AMK_MODE == 0

#define AMK_TORQUE_LIM 2143

typedef struct
{
	uint32 amk_ac1;
	uint32 amk_ac2;
	uint32 amk_set;
} id_set_t;

typedef union {
	uint32 TransmitData[2];
	struct
	{
		boolean inv1_error : 1;            // [0]
		boolean inv1_warn : 1;             // [1]
		boolean inv1_system_ready : 1;     // [2]
		boolean inv1_quit_dc_on : 1;       // [3]
		boolean inv1_ef_on : 1;            // [4]
		boolean inv1_be1_on : 1;           // [5]
		boolean inv1_quit_inverter_on : 1; // [6]
		boolean inv1_be2_on : 1;           // [7]

		boolean inv2_error : 1;            // [8]
		boolean inv2_warn : 1;             // [9]
		boolean inv2_system_ready : 1;     // [10]
		boolean inv2_quit_dc_on : 1;       // [11]
		boolean inv2_ef_on : 1;            // [12]
		boolean inv2_be1_on : 1;           // [13]
		boolean inv2_quit_inverter_on : 1; // [14]
		boolean inv2_be2_on : 1;           // [15]

		boolean inv3_error : 1;            // [16]
		boolean inv3_warn : 1;             // [17]
		boolean inv3_system_ready : 1;     // [18]
		boolean inv3_quit_dc_on : 1;       // [19]
		boolean inv3_ef_on : 1;            // [20]
		boolean inv3_be1_on : 1;           // [21]
		boolean inv3_quit_inverter_on : 1; // [22]
		boolean inv3_be2_on : 1;           // [23]

		boolean inv4_error : 1;            // [24]
		boolean inv4_warn : 1;             // [25]
		boolean inv4_system_ready : 1;     // [26]
		boolean inv4_quit_dc_on : 1;       // [27]
		boolean inv4_ef_on : 1;            // [28]
		boolean inv4_be1_on : 1;           // [29]
		boolean inv4_quit_inverter_on : 1; // [30]
		boolean inv4_be2_on : 1;           // [31]

		uint32 reserve : 32; // [32-63]
	} S;
} inv_seq_t;

typedef union {
	uint32 RecievedData[2];
	struct
	{
		boolean inv1_ef_on : 1;  // [0]
		boolean inv1_be1_on : 1; // [1]
		boolean inv1_be2_on : 1; // [2]
		boolean inv1_ba3_on : 1; // [3]

		boolean inv2_ef_on : 1;  // [4]
		boolean inv2_be1_on : 1; // [5]
		boolean inv2_be2_on : 1; // [6]
		boolean inv2_ba3_on : 1; // [7]

		boolean inv3_ef_on : 1;  // [8]
		boolean inv3_be1_on : 1; // [9]
		boolean inv3_be2_on : 1; // [10]
		boolean inv3_ba3_on : 1; // [11]

		boolean inv4_ef_on : 1;  // [12]
		boolean inv4_be1_on : 1; // [13]
		boolean inv4_be2_on : 1; // [14]
		boolean inv4_ba3_on : 1; // [15]

		uint16 reserve_0 : 16; // [16-31]
		uint32 reserve_1 : 32; // [32-63]
	} S;
} inv_status_t;

typedef union {
	uint32 RecievedData[2];
	struct
	{
		uint8 AMK_bReserve : 8;             // [0-7]
		boolean AMK_bSystemReady : 1;       // [8]
		boolean AMK_bError : 1;             // [9]
		boolean AMK_bWarn : 1;              // [10]
		boolean AMK_bQuitDcOn : 1;          // [11]
		boolean AMK_bDcOn : 1;              // [12]
		boolean AMK_bQuitInverterOn : 1;    // [13]
		boolean AMK_bInverterOn : 1;        // [14]
		boolean AMK_bDerating : 1;          // [15]
		sint16 AMK_ActualVelocity : 16;     // [16-31]
		sint16 AMK_TorqueCurrent : 16;      // [32-47]
		sint16 AMK_MagnetizingCurrent : 16; // [48-63]
	} S;
} amk_actual_values_1_t;

typedef union {
	uint32 RecievedData[2];
	struct
	{
		sint16 AMK_TempMotor : 16;    // [0-15]
		sint16 AMK_TempInverter : 16; // [16-31]
		uint16 AMK_ErrorInfo : 16;    // [32-47]
		sint16 AMK_TempIGBT : 16;     // [48-63]
	} S;
} amk_actual_values_2_t;

typedef union {
	uint32 TransmitData[2];
	struct
	{
		uint8 AMK_bReserve1 : 8;            // [0-7]
		boolean AMK_bInverterOn : 1;        // [8]
		boolean AMK_bDcOn : 1;              // [9]
		boolean AMK_bEnable : 1;            // [10]
		boolean AMK_bErrorReset : 1;        // [11]
		uint8 AMK_bReserve2 : 4;            // [12-15]
		sint16 AMK_TargetVelocity : 16;     // [16-31]
		sint16 AMK_TorqueLimitPositv : 16;  // [32-47]	//0.1Mn
		sint16 AMK_TorqueLimitNegativ : 16; // [48-63]	//0.1Mn
	} S;
} amk_setpoint_1_t;

typedef struct
{
	boolean error_reset;
	boolean dc_on;
	sint16 torque_limit_negativ;
	sint16 torque_limit_positv;
	boolean ef_on;
	boolean be1_on;
	boolean enable;
	boolean inverter_on;
	boolean be2_on;
	sint16 target_velocity;
} inv_switch_t;

typedef struct
{
	boolean inv_on;

	uint8 inv_address;
	id_set_t inv_id;
	amk_setpoint_1_t amk_setpoint_1;
	amk_actual_values_1_t amk_actual_values_1;
	amk_actual_values_2_t amk_actual_values_2;
	inv_switch_t inv_switch;
	uint32 inv_seq_timer;
	uint32 inv_error_reset_cnt;

	CanCommunication_Message t_amk_setpoint_1;
	CanCommunication_Message r_amk_actual_values_1;
	CanCommunication_Message r_amk_actual_values_2;

	id_set_t inv_id_log;

	CanCommunication_Message t_amk_setpoint_1_log;
	CanCommunication_Message t_amk_actual_values_1_log;
	CanCommunication_Message t_amk_actual_values_2_log;
} private_inv_t;

typedef struct
{
	uint32 id;
	inv_seq_t inv_seq;

	CanCommunication_Message t_inv_seq_node_1;
	CanCommunication_Message t_inv_seq_node_2;
} private_inv_seq_t;

typedef struct
{
	uint32 id;
	inv_status_t inv_status;

	CanCommunication_Message r_inv_status_node_1;
	CanCommunication_Message r_inv_status_node_2;
} private_inv_status_t;

typedef struct
{
	boolean r2d;

	sint16 fl;
	sint16 fr;
	sint16 rl;
	sint16 rr;

	float32 spdFl;
	float32 spdFr;
	float32 spdRl;
	float32 spdRr;

	boolean brakeOn;

	boolean acceleraing;

	IfxCpu_mutexLock mutex;
} AmkInverterPublic_t;

struct Monitor
{
	struct
	{
		uint16 error_FL;
		uint16 error_RL;
		uint16 error_RR;
		uint16 error_FR;
	} InverterErrorState;
	struct
	{
		sint16 temp_FL;
		sint16 temp_RL;
		sint16 temp_RR;
		sint16 temp_FR;
	} MotorTemp;
	struct
	{
		sint16 temp_FL;
		sint16 temp_RL;
		sint16 temp_RR;
		sint16 temp_FR;
	} InverterTemp;
	struct
	{
		sint16 velocity_FL;
		sint16 velocity_RL;
		sint16 velocity_RR;
		sint16 velocity_FR;
	} MotorVelocity;
};

typedef struct
{
	struct Monitor monitor;
	IfxCpu_mutexLock mutex;
} AmkInverterMonitorPublic_t;

IFX_EXTERN AmkInverterPublic_t AmkInverterPublic;
IFX_EXTERN AmkInverterMonitorPublic_t AmkInverterMonitorPublic;

IFX_EXTERN void AmkInverter_can_init(void);
IFX_EXTERN void AmkInverter_can_Run(void);
IFX_EXTERN void AmkInverter_Start(boolean rtd_flag);
IFX_EXTERN void AmkInverter_writeMessageFront(sint16 torque_left, sint16 torque_right, boolean accelerating);
IFX_EXTERN void AmkInverter_writeMessageRear(sint16 torque_left, sint16 torque_right, boolean accelerating);
#else
#define AMK_TORQUE_LIM 2143

typedef struct
{
	uint8 AMK_bReserve : 8;
	boolean AMK_bSystemReady : 1;
	boolean AMK_bWarn : 1;
	boolean AMK_bSError : 1;
	boolean AMK_bQuitDcOn : 1;
	boolean AMK_bDcOn : 1;
	boolean AMK_bQuitInverterOn : 1;
	boolean AMK_bInverterOn : 1;
	boolean AMK_bDerating : 1;
} AMK_Status;

typedef struct
{
	uint8 AMK_bReserve1 : 8;
	boolean AMK_bInverterOn : 1;
	boolean AMK_bDcOn : 1;
	boolean AMK_bEnable : 1;
	boolean AMK_bErrorReset : 1;
	uint8 AMK_bReserve2 : 4;
} AMK_Control;

typedef union {
	uint32 RecievedData[2];
	struct
	{
		uint8 AMK_bReserve : 8;
		boolean AMK_bSystemReady : 1;
		boolean AMK_bSError : 1;
		boolean AMK_bWarn : 1;
		boolean AMK_bQuitDcOn : 1;
		boolean AMK_bDcOn : 1;
		boolean AMK_bQuitInverterOn : 1;
		boolean AMK_bInverterOn : 1;
		boolean AMK_bDerating : 1;
		sint16 AMK_ActualVelocity : 16;
		sint16 AMK_TorqueCurrent : 16;
		sint16 AMK_MagnetizingCurrent : 16;
	} S;
} amkActualValues1;

typedef union {
	uint32 RecievedData[2];
	struct
	{
		sint16 AMK_TempMotor : 16;
		sint16 AMK_TempInverter : 16;
		uint16 AMK_ErrorInfo : 16;
		sint16 AMK_TempIGBT : 16;
	} S;
} amkActualValues2;

typedef union {
	uint32 TransmitData[2];
	struct
	{
		uint8 AMK_bReserve1 : 8;
		boolean AMK_bInverterOn : 1;
		boolean AMK_bDcOn : 1;
		boolean AMK_bEnable : 1;
		boolean AMK_bErrorReset : 1;
		uint8 AMK_bReserve2 : 4;
		sint16 AMK_Torque_setpoint : 16; // 0.1Mn
		sint16 AMK_TorqueLimitPositv : 16;
		sint16 AMK_TorqueLimitNegativ : 16;
	} S;
} amkSetpoint1;

typedef union {
	uint32 TransmitData[2];
	struct
	{
		uint16 EFon;
		uint16 BE1on;
		uint16 BE2on;
		uint16 Remain;
	} B;
} Inv_switch_msg_t;

typedef struct
{
	uint16 ID_AMK_Ac1;
	uint16 ID_AMK_Ac2;
	uint16 ID_AMK_Set;
} ID_set;

typedef enum AmkState_e
{
	AmkState_S0 = 0, // Power On
	AmkState_S1 = 1, // System Ready
	AmkState_S2 = 2, // DC On
	AmkState_S3 = 3, // EF on
	AmkState_S4 = 4, // BE1 on
	AmkState_S5 = 5, // Enable/InverterEnable on
	AmkState_RTD     // Ready To Drive
} AmkState_t;

typedef struct
{
	AmkState_t r2d;

	float32 fl;
	float32 fr;
	float32 rl;
	float32 rr;

	float32 spdFl;
	float32 spdFr;
	float32 spdRl;
	float32 spdRr;

	boolean brakeOn;

	IfxCpu_mutexLock mutex;
} AmkInverterPublic_t;

struct Monitor
{
	int InverterTemp;
	struct
	{
		uint16 error_FL;
		uint16 error_RL;
		uint16 error_RR;
		uint16 error_FR;
	} InverterErrorState;
	struct
	{
		sint16 temp_FL;
		sint16 temp_RL;
		sint16 temp_RR;
		sint16 temp_FR;
	} MotorTemp;
	struct
	{
		sint16 velocity_FL;
		sint16 velocity_RL;
		sint16 velocity_RR;
		sint16 velocity_FR;
	} MotorVelocity;
	// struct MotorCurrent{
	//     uint16 velocity_RL;
	//     uint16 velocity_FL;
	//     uint16 velocity_RR;
	//     uint16 velocity_FR;
	// }
};

typedef struct
{
	struct Monitor monitor;
	IfxCpu_mutexLock mutex;
} AmkInverterMonitorPublic_t;

IFX_EXTERN AmkInverterPublic_t AmkInverterPublic;

IFX_EXTERN AmkInverterMonitorPublic_t AmkInverterMonitorPublic;

IFX_EXTERN Inv_switch_msg_t Inv_switch_msg;
IFX_EXTERN amkActualValues1 INV_FL_AMK_Actual_Values1;
IFX_EXTERN amkActualValues1 INV_RL_AMK_Actual_Values1;
IFX_EXTERN amkActualValues1 INV_RR_AMK_Actual_Values1;
IFX_EXTERN amkActualValues1 INV_FR_AMK_Actual_Values1;

IFX_EXTERN amkActualValues2 INV_FL_AMK_Actual_Values2;
IFX_EXTERN amkActualValues2 INV_RL_AMK_Actual_Values2;
IFX_EXTERN amkActualValues2 INV_RR_AMK_Actual_Values2;
IFX_EXTERN amkActualValues2 INV_FR_AMK_Actual_Values2;

IFX_EXTERN amkSetpoint1 INV1_AMK_Setpoint1;
IFX_EXTERN amkSetpoint1 INV2_AMK_Setpoint1;
IFX_EXTERN amkSetpoint1 INV3_AMK_Setpoint1;
IFX_EXTERN amkSetpoint1 INV4_AMK_Setpoint1;

IFX_EXTERN ID_set Inverter1;
IFX_EXTERN ID_set Inverter2;
IFX_EXTERN ID_set Inverter3;
IFX_EXTERN ID_set Inverter4;

IFX_EXTERN AmkState_t AmkState;

IFX_EXTERN void AmkInverter_can_init(void);
IFX_EXTERN void AmkInverter_can_Run(void);
IFX_EXTERN void AmkInverter_can_write(amkSetpoint1 *INV, CanCommunication_Message TC, uint16 tV);
IFX_EXTERN void AmkInverter_writeMessage(uint16 Value1, uint16 Value2);
IFX_EXTERN void AmkInverter_writeMessage2(uint16 Value1, uint16 Value2);
IFX_EXTERN void InverterControlSet();
IFX_EXTERN void AmkInverter_Start(boolean rtdFlag);
#endif

#endif /* AMKINVERTER_CAN_H_ */
