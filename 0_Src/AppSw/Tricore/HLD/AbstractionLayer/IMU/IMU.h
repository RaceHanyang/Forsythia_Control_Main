#ifndef SRC_APPSW_TRICORE_HLD_IMU_IMU_H_
#define SRC_APPSW_TRICORE_HLD_IMU_IMU_H_

#include <stdbool.h>

#include <Ifx_Types.h>
#include "Configuration.h"
#include "ConfigurationIsr.h"
#include "HLD.h"

#include "CanCommunication.h"


typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 free_acc_x;   //unit: 2^{-8}m/s^2
        uint16 free_acc_y;   //unit: 2^{-8}m/s^2
        uint16 free_acc_z;   //unit: 2^{-8}m/s^2
        uint16 reserved_0;
	} s;
}free_acc_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 gyr_x;    //unit: 2^{-9}rad/s
        uint16 gyr_y;    //unit: 2^{-9}rad/s
        uint16 gyr_z;    //unit: 2^{-9}rad/s
        uint16 reserved_0;
	} s;
}gyr_t;


typedef struct
{
        float free_acc_x_value; //unit: m/s^2
        float free_acc_y_value; //unit: m/s^2
        float free_acc_z_value; //unit: m/s^2

        float gyr_x_value; //unit: rad/s
        float gyr_y_value; //unit: rad/s
        float gyr_z_value; //unit: rad/s
}IMU_value_t;


typedef struct 
{
    free_acc_t      free_acc;
    gyr_t           gyr;

    IMU_value_t IMU_value;

    CanCommunication_Message free_acc_msg;
    CanCommunication_Message gyr_msg;
}IMU_t;

IFX_EXTERN IMU_t IMU;

IFX_EXTERN void IMU_CAN_init(void);
IFX_EXTERN void IMU_run_1ms(void);

#endif

