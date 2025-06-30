#include "powerlimit.h"
#include "RVC.h"

#define POWER_LIMIT             (80000-10000)   //80-1 = 79kW (1KW Margin)
#define CURRENT_LIM_SET_VAL		10		        //10A
#define Kt                      0.26            //Torque Constant. Current * Kt = Torque

#define Kp 1     //Proportional term Gain
#define Ki 1     //Integral term Gain
#define Kd 1     //Derivative term Gain
#define dt 0.001 // Unit : sec, Same as Period of PID control

power_limit_t power_limit;
extern RVC_t RVC;

void PowerLimit_init(void);
void POWERLIMIT_run_1ms(void);
IFX_STATIC void PowerComputation(void);
IFX_STATIC void TorqueLimit(void);
IFX_STATIC float PIDComputation(float);


void PowerLimit_init(void) {
    power_limit.power_limit = POWER_LIMIT;
}

void POWERLIMIT_run_1ms(void)
{
    void PowerComputation();
    void TorqueLimit();
}

IFX_STATIC void PowerComputation(void)
{
	power_limit.power_value = RVC_public.bms.data.current * RVC_public.bms.data.voltage;
	power_limit.current_limit = RVC.power.limit / RVC_public.bms.data.voltage;
}

IFX_STATIC float PIDComputation(float error)
{
    power_limit.PID.integral += (error * dt);
    
    power_limit.PID.derivative = (error - (power_limit.PID.prev_error)) / dt;

    power_limit.PID.prev_error = error;

    power_limit.PID.output = (Kp * error) + (Ki * (power_limit.PID.integral)) + (Kd * (power_limit.PID.derivative));

    return power_limit.PID.output;
}

IFX_STATIC void TorqueLimit(void)
{
    if(power_limit.power_value <= POWER_LIMIT)
    {
        power_limit.power_over = 0;
    }
    else if(power_limit.power_value > POWER_LIMIT)
    {
        power_limit.power_over = ((power_limit.power_value) - POWER_LIMIT);
    }
    
    power_limit.current_over = (power_limit.power_over)/(RVC_public.bms.data.voltage); 

    power_limit.torque_over = Kt * (power_limit.current_over);

    power_limit.torque_error = PIDComputation(power_limit.torque_over);

    RVC.torque.controlled = (RVC.torque.controlled) - (power_limit.torque_error);
}





