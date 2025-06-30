#ifndef POWERLIMIT_H_
#define POWERLIMIT_H_

#include "RVC.h"

IFX_EXTERN void POWERLIMIT_run_1ms(void);

typedef struct 
{
    uint16 power_value      :16;
    uint16 current_limit    :16;
    uint16 current_over     :16;
    uint16 
}POWERLIMIT_t;


#endif
