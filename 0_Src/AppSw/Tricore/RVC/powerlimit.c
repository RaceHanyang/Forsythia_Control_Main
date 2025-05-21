#include "powerlimit.h"
#include "RVC.h"

#define POWER_LIMIT (80000-1000) //Unit : Watt
#define Kt 0.26 //Torque Constant. Current * Kt = Torque

#define Kp 1 //Proportional term Gain
#define Ki 1 //Integral term Gain
#define Kd 1 //Derivative term Gain
#define dt 0.001 // Unit : sec, Same as Period of PID control

RVC.power.limit = POWER_LIMIT;

void power_computation(void);
void torque_limit(void);
void PID_computation(void);
void 

IFX_STATIC void power_computation(void)
{
	RVC.power.value = RVC_public.bms.data.current * RVC_public.bms.data.voltage;
	RVC.power.currentLimit = RVC.power.limit / RVC_public.bms.data.voltage;
}

IFX_STATIC float PID_computation(float error)
{
    float integral;
    float derivative;
    float prev_error;

    integral = integral + (error * dt);
    
    derivative = (error - pid->prev_error) / dt;

    pid->prev_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    return output;
}

IFX_STATIC void torque_limit(void)
{
    float power_over;
    if(RVC.power.value <= POWER_LIMIT)
    {
        power_over = 0
    }
    else if(RVC.power.value > POWER_LIMIT)
    {
        power_over = (RVC.power.value - POWER_LIMIT);
    }
    
    float current_over;
    current_over = power_over/RVC_public.bms.data.voltage; 

    float torque_over;
    torque_over = Kt * current_over;

    float torque_error = PID_computation(torque_over);

    RVC.torque.controlled = RVC.torque.controlled - torque_error;
}

void POWERLIMIT_run_1ms(void)
{
    void power_computation();
    void torque_limit();
}