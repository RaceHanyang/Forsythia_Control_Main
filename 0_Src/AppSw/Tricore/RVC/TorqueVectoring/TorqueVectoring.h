/*
 * TorqueVectoring.h
 * Created on 2019. 12. 10.
 * Author: Dua
 */

#ifndef TORQUEVECTORING_H_
#define TORQUEVECTORING_H_

/* Includes */
#include "RVC.h"
#include "RVC_privateDataStructure.h"

/*************************** Data Structures *********************************/
typedef struct
{
	struct
	{
        float v_x;  // enable limit of velocity
        float delta_sta;    // enable limit of steering angle
        float k_1;  // gain for enable limit of steering angle
	} enable;

    struct
    {
        float k_2;  // (steering angle) - (wheel gear ratio)
        float i;    // |(front accel) - (center of mass)|
    } error;

    struct
    {
        float k_p;  // proportional gain
        float k_i;  // integral gain
        float k_d;  // derivative gain
        float k_b;  // inverse calculation constant == k_i
    } pid;

    struct
    {
        float max_delta_trq;    // limit of maximum delta torque
    } limit;

    struct
    {
        float t;    // constant of transfer function constant
    } tv;
    
} TV_t;

/* Functionc Prototypes */
IFX_EXTERN void RVC_TorqueVectoring_run_modeOpen(void);
IFX_EXTERN void RVC_TorqueVectoring_run_mode1(void);
#endif