#include "powerlimit.h"

//* Defines *//
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define CLAMP(min, max, value) (MAX(min,(MIN(value, max))))

#define POWER_LIMIT             (80000-10000)   //80-1 = 79kW        (1KW Margin)
#define V_epsilon               0.001           //					 (Unit : M/s)
#define F_max                   2143            //					 (Unit : 10mN/M)
#define K_p                     (1/P_band) 
#define P_band                  5               //Linear decay range (Uint : kW)     
#define T_s                     1000            //Sampling Period    (Uint : Hz)
#define t                       0.05            //Time constant      (Unit : sec)

//* Structures *//
power_limit_t power_limit;


//* Function implementations *//
void PowerLimit_run_1ms(void);

IFX_STATIC void CalculateAlpha(void);
IFX_STATIC float CalculateFeedForward(float32 velocity);
IFX_STATIC float CalculateFeedBack(float32 current, float32 voltage);
IFX_STATIC float LPF(float32 input);

//* Functions *//
void PowerLimit_run_1ms(void)
{
    CalculateAlpha();
}

void CalculateAlpha(void)
{   
    float32 velocity = ((AmkInverterMonitorPublic.monitor.MotorVelocity.velocity_FL +
                        AmkInverterMonitorPublic.monitor.MotorVelocity.velocity_FR +
                        AmkInverterMonitorPublic.monitor.MotorVelocity.velocity_RL +
                        AmkInverterMonitorPublic.monitor.MotorVelocity.velocity_RR) / 4);
    power_limit.alpha_ff = CalculateFeedForward(velocity);
    power_limit.alpha_fb = CalculateFeedBack(RVC_public.bms.shared.data.current, RVC_public.bms.shared.data.voltage);
    power_limit.alpha = (power_limit.alpha_ff * power_limit.alpha_fb);
}

IFX_STATIC float CalculateFeedForward(float32 velocity)
{
	float32 F_lim = (POWER_LIMIT)/(MAX(velocity, V_epsilon));
    float32 alpha_ff = MIN((4*F_max),F_lim)/(4*F_max); //feed forward

    return alpha_ff;
}

IFX_STATIC float CalculateFeedBack(float32 current, float32 voltage)
{
    float32 epsilon_p = (POWER_LIMIT - (current * voltage));
    float32 alpha_raw = CLAMP(0,1,(K_p*epsilon_p));
    float32 alpha_fb = LPF(alpha_raw); //feed back

    return alpha_fb;
}

IFX_STATIC float LPF(float32 input)
{
    float32 alpha =(t/(t+T_s)); 
    float32 prev_input, output;

    output = (alpha * prev_input) + ((1-alpha) * input);

    prev_input = input;

    return output;
}


