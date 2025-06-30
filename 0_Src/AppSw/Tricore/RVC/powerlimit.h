#ifndef POWERLIMIT_H_
#define POWERLIMIT_H_

#include "RVC.h"
#include "RVC_privateDataStructure.h"

IFX_EXTERN void POWERLIMIT_run_1ms(void);
typedef struct
{
    float integral;
    float derivative;
    float prev_error;
    float output;
}PID_t;
typedef struct 
{
    uint32 power_limit;
    float current_limit;
    float current_over;     
    float power_over;       
    float torque_over;      
    float power_value;      
    float torque_error;     
    PID_t PID;
}power_limit_t;



#endif
