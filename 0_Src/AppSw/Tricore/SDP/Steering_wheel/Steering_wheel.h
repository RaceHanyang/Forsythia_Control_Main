#ifndef STEERINGWHEEL_H
#define STEERINGWHEEL_H
/**************************** Includes *******************************/
#include "CanCommunication.h"

/***************************** Macro *********************************/


/************************* Data Structures ***************************/

CanCommunication_Message steering_wheel_msg;

// id 0x30300
// freq 100ms
typedef union
{
  uint32 data[2];
  struct {
    uint8 LeftRotary    :  8;
    uint8 RightRotary   :  8;
    uint8 SW1 		    :  8;
    uint8 SW2 		    :  8;
    uint8 SW3 		    :  8;
    uint8 SW4 		    :  8;
    uint8 reserved1	    :  8;
    uint8 reserved2	    :  8;
  } s;
} steering_wheel_t;

#endif
