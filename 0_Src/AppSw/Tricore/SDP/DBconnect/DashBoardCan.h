#ifndef DASHBOARDCAN_H_
#define DASHBOARDCAN_H_

#include "HLD.h"
#include <Port/Io/IfxPort_Io.h>
#include "Configuration.h"
#include "CanCommunication.h"
#include "Gpio_Debounce.h"
#include "AmkInverter_can.h"
#include "Platform_Types.h"


typedef union{
	uint32 RxData[2];
	struct{
		uint8 StartBtnPushed : 1;
        uint8 OFFvehicle : 1;
		uint32 Remain1 : 30;
        uint32 Remain2;
	}B;

}StartBtnPushed_t;

typedef union 
{
	uint32 data[2];				//[0-31][32-63]
	struct 
	{
		boolean 		tsal_on                	:1;  // [0]
        boolean 		bms_ok                 	:1;  // [1]
        boolean 		imd_ok                 	:1;  // [2]
        boolean 		bspd_ok                	:1;  // [3]
        boolean 		apps_ok                	:1;  // [4]
        boolean 		bpps_ok                	:1;  // [5]
        boolean 		sdc_ok                	:1;  // [6]
        boolean 		rtd_on                 	:1;  // [7]
        uint8 			reserved_0				:8;  // [8-15]
        uint16 			start_cnt				:16; // [16-31]
		uint32 			reserved_1				:32; // [32-63]
	}B;
} DashBoardMsg0_t;

typedef union 
{
	uint32 data[2];				//[0-31][32-63]
	struct 
	{
        boolean 		start_up                :1;  // [0]
        boolean			bms_ok					:1;	 // [1]
        boolean 		imd_ok                 	:1;  // [2]
        boolean 		bspd_ok                	:1;  // [3]
        boolean 		apps_ok                	:1;  // [4]
        boolean 		bpps_ok                	:1;  // [5]
        boolean 		sdc_ok                	:1;  // [6]
        boolean 		rtd_on                 	:1;  // [7]
        uint8 			reserved_0				:8;  // [8-15]
        uint16 			start_cnt_mirror		:16; // [16-31]
		uint32 			reserved_1				:32; // [32-63]
	}B;
} DashBoardMsg1_t;

typedef struct 
{
	boolean vcuOk;
	boolean bmsOk;
	boolean imdOk;
	boolean bspdOk;
	boolean appsOk;
	boolean bppsOk;
	boolean sdcSenFinal;
	boolean rtdOn;
	boolean brakeOn;
	boolean tsalOn;
}DashBoard_info_t;

typedef struct
{
	DashBoard_info_t data;
	struct 
	{
		DashBoard_info_t data;
		IfxCpu_mutexLock mutex;
	}shared;
}DashBoard_public_t;

IFX_EXTERN StartBtnPushed_t StartBtnPushed;
IFX_EXTERN StartBtnPushed_t StartBtnMirror;

IFX_EXTERN DashBoard_public_t DashBoard_public;
IFX_EXTERN boolean RTD_flag;

IFX_EXTERN void SDP_DashBoardCan_init(void);
IFX_EXTERN void SDP_DashBoardCan_run_1ms(void);
IFX_EXTERN void SDP_DashBoardCan_run_10ms(void);

#endif
