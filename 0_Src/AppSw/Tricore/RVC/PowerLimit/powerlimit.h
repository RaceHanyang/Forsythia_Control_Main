#ifndef POWERLIMIT_H_
#define POWERLIMIT_H_

#include "RVC.h"
#include "RVC_privateDataStructure.h"

typedef struct 
{
    float32 alpha_ff;
    float32 alpha_fb;

    float32 alpha;
}power_limit_t;


#endif
