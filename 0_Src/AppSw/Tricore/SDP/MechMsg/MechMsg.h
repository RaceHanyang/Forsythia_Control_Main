/*
 * MechMsg.h
 *
 *  Created on: 2025. 2. 25.
 *      Author: 82104
 */

#ifndef MECH_MSG_H
#define MECH_MSG_H

/**************************** Includes *******************************/
#include "CanCommunication.h"

/***************************** Macro *********************************/


/************************* Data Structures ***************************/
typedef union
{
    uint32 data[2];
    struct
    {
        uint16 	steering_angel     :16;    //[0-15]
        uint8 	apps               :8;     //[16-23]
        uint8 	bpps               :8;     //[24-31]
        uint16 	brake_pressure_0   :16;    //[32-47]
        uint16 	brake_pressure_1   :16;    //[48-63]
    } s;
} steering_and_pedal_t;

typedef struct
{
	steering_and_pedal_t steering_and_pedal;

	CanCommunication_Message steering_and_pedal_msg;

} mech_msg_t;

/************************ Global Variables ***************************/
IFX_EXTERN mech_msg_t mech_msg;

/*********************** Function Prototypes *************************/
IFX_EXTERN void SDP_MechMsg_init(void);
IFX_EXTERN void SDP_MechMsg_run_1000ms(void);



#endif
