#include "powerlimit.h"
#include "RVC.h"

#define POWER_LIMIT 79000 //Watt
RVC.power.limit = POWER_LIMIT;

void power_computation(void);
void get_torque_controlled(void);
void power_limit(void);

IFX_INLINE void RVC_powerComputation(void)
{
	RVC.power.value = RVC_public.bms.data.current * RVC_public.bms.data.voltage;
	RVC.power.currentLimit = RVC.power.limit / RVC_public.bms.data.voltage;
}

IFX_STATIC void power_limit(void)
{
    int power_over;
    if(RVC.power.value <= POWER_LIMIT)
    {
        power_over = 0
    }
    else if(RVC.power.value > POWER_LIMIT)
    {
        power_over = (RVC.power.value - POWER_LIMIT);
    }
    
    int current_over;
    current_over = power_over/RVC_public.bms.data.voltage; 

    
    

}

