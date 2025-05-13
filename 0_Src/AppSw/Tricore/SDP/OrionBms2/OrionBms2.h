/*
 * OrionBms2.h
 * Created on: 2020.08.04
 * Author: Dua
 */

#ifndef ORIONBMS2_H
#define ORIONBMS2_H
/**************************** Includes *******************************/
#include "CanCommunication.h"

/***************************** Macro *********************************/


/************************* Data Structures ***************************/
// id 0x300
// freq 8ms
// ccl: charge current limit
// dcl: discharge current limit
typedef union // power
{
    uint32 data[2];
    struct
    {
        uint16 current                   :16;    //[0-15]
        uint16 voltage                   :16;    //[16-31]
        uint16 ccl                       :16;    //[32-47] 
        uint16 dcl                       :16;    //[48-63]
    } s;
} pack_pow_t;

// id 0x20000
// freq 1000ms
// cl: current limit
typedef union
{
    uint32 data[2];
    struct
    {
        uint16 failsafe_status           :16;    //[0-15]
        uint16 dtc_status_1              :16;    //[16-31]
        uint16 dtc_status_2              :16;    //[32-47]
        uint16 cl_status                 :16;    //[48-63]
    } s;
} pack_status_t;

// id 0x40000
// freq 104ms
typedef union // cell voltage
{
    uint32 data[2];
    struct
    {
        uint8  highest_cv_id             :8;     //[0-7]
        uint16 highest_cv                :16;    //[8-23]
        uint16 average_cv                :16;    //[24-39]
        uint8  lowest_cv_id              :8;     //[40-47]
        uint16 lowest_cv                 :16;    //[48-63]
    } s;
} pack_cv_t;

// id 0x40001
// freq 104ms
typedef union // open cell voltage
{
    uint32 data[2];
    struct
    {
        uint8  highest_ocv_id            :8;     //[0-7]
        uint16 highest_ocv               :16;    //[8-23]
        uint16 average_ocv               :16;    //[24-39]
        uint8  lowest_ocv_id             :8;     //[40-47]
        uint16 lowest_ocv                :16;    //[48-63]
    } s;
} pack_ocv_t;

// id 0x40002
// freq 104ms
typedef union
{
    uint32 data[2];
    struct
    {
        uint8  highest_therm_id          :8;     //[0-7]
        uint8  highest_temp              :8;     //[8-15]
        uint8  average_temp              :8;     //[16-23]
        uint8  lowest_therm_id           :8;     //[24-31]
        uint8  lowest_temp               :8;     //[32-39]
        uint16 resistance                :16;    //[40-55]
        uint8  reserved                  :8;     //[56-63]
    } s;
} pack_temp_t;

// id 0x40003
// freq 1000ms
typedef union // state of charge
{
    uint32 data[2];
    struct
    {
        uint8  soc                       :8;     //[0-7]
        uint8  adaptive_soc              :8;     //[8-15]
        uint16 adaptivc_tot_cap          :16;    //[16-31]
        uint16 open_voltage              :8;     //[32-47]
        uint16 reserved                  :16;    //[48-63]
    } s;
} pack_soc_t;

typedef struct 
{
	pack_pow_t pack_pow;
	pack_status_t pack_status;
	pack_cv_t pack_cv;
	pack_ocv_t pack_ocv;
	pack_temp_t pack_temp;
	pack_soc_t pack_soc;

	CanCommunication_Message pack_pow_msg;
	CanCommunication_Message pack_status_msg;
	CanCommunication_Message pack_cv_msg;
	CanCommunication_Message pack_ocv_msg;
	CanCommunication_Message pack_temp_msg;
	CanCommunication_Message pack_soc_msg;

	uint32 canErrorCount;
	boolean canError;
}OrionBms2_t;

/************************ Global Variables ***************************/
IFX_EXTERN OrionBms2_t OrionBms2;

/*********************** Function Prototypes *************************/
IFX_EXTERN void OrionBms2_init(void);
IFX_EXTERN void OrionBms2_run_1ms_c2(void);

#endif
