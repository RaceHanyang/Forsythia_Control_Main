#ifndef SRC_APPSW_TRICORE_HLD_NEWIMU_NEWIMU_H_
#define SRC_APPSW_TRICORE_HLD_NEWIMU_NEWIMU_H_

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
	uint8  year;    //unit: year
        uint8  month;   //unit: month   
        uint8  day;     //unit: day
        uint8  hour;    //unit: hour
        uint8  min;     //unit: min
        uint8  sec;     //unit: sec
        uint16 tenthms; //unit: sec
	} s;
}utc_t;

typedef union 
{
	uint32 data[2];
	struct
	{
	uint8  error_code;
        uint8  reserved_0;
        uint16 reserved_1;
        uint16 reserved_2;
        uint16 reserved_3;
	} s;
}imu_err_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint32 status_word;
        uint32 reserved_0;
	} s;
}imu_status_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 acc_x;   //unit: 2^{-8}m/s^2
        uint16 acc_y;   //unit: 2^{-8}m/s^2
        uint16 acc_z;   //unit: 2^{-8}m/s^2
        uint16 reserved_0;
	} s;
}free_acc_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 acc_x;   //unit: 2^{-8}m/s^2
        uint16 acc_y;   //unit: 2^{-8}m/s^2
        uint16 acc_z;   //unit: 2^{-8}m/s^2
        uint16 reserved_0;
	} s;
}acc_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 vel_x;   //unit: 2^{-6}m/s
        uint16 vel_y;   //unit: 2^{-6}m/s
        uint16 vel_z;   //unit: 2^{-6}m/s
        uint16 reserved_0;
	} s;
}velocity_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 roll;    //unit: 2^{-7}deg
        uint16 pitch;   //unit: 2^{-7}deg
        uint16 yaw;     //unit: 2^{-7}deg
        uint16 reserved_0;
	} s;
}euler_angle_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint16 gry_x;    //unit: 2^{-9}rad/s
        uint16 gry_y;    //unit: 2^{-9}rad/s
        uint16 gry_z;    //unit: 2^{-9}rad/s
        uint16 reserved_0;
	} s;
}gyr_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint32 lat;     //unit: 2^{-24}deg
        uint32 lon;     //unit: 2^{-24}deg
	} s;
}coordi_t;

typedef union 
{
	uint32 data[2];
	struct
	{
        uint32 alt_ellipsoid; //unit: 2^{-15}m
        uint32 reserved_0;
	} s;
}altitude_t;

typedef struct 
{
    utc_t           utc;
    imu_err_t       imu_err;
    imu_status_t    imu_status;
    free_acc_t      free_acc;
    acc_t           acc;
    velocity_t      velocity;
    euler_angle_t   euler_angle;
    gyr_t           gyr;
    coordi_t        coordi;
    altitude_t      altitude;

    CanCommunication_Message utc_msg;
    CanCommunication_Message imu_err_msg;
    CanCommunication_Message imu_status_msg;
    CanCommunication_Message free_acc_msg;
    CanCommunication_Message acc_msg;
    CanCommunication_Message velocity_msg;
    CanCommunication_Message euler_angle_msg;
    CanCommunication_Message gyr_msg;
    CanCommunication_Message coordi_msg;
    CanCommunication_Message altitude_msg;
}new_IMU_t;

IFX_EXTERN new_IMU_t new_IMU;

IFX_EXTERN void New_IMU_CAN_init(void);
IFX_EXTERN void New_IMU_run_10ms(void);

#endif

