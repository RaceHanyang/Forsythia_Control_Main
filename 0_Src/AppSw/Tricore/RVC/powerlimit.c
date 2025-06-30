#include "powerlimit.h"
#include "RVC.h"

#define POWER_LIMIT             (80000-10000)   //80-1 = 79kW (1KW Margin)
#define CURRENT_LIM_SET_VAL		10		        //10A
#define Kt                      0.26            //Torque Constant. Current * Kt = Torque

#define Kp 1     //Proportional term Gain
#define Ki 1     //Integral term Gain
#define Kd 1     //Derivative term Gain
#define dt 0.001 // Unit : sec, Same as Period of PID control

POWERLIMIT_t POWERLIMIT;

POWERLIMIT.power_limit = POWER_LIMIT;

void PowerComputation(void);
void TorqueLimit(void);
void PIDComputation(void);

IFX_STATIC void PowerComputation(void)
{
	POWERLIMIT.power_value = RVC_public.bms.data.current * RVC_public.bms.data.voltage;
	POWERLIMIT.current_imit = RVC.power.limit / RVC_public.bms.data.voltage;
}

IFX_STATIC float PIDComputation(float error)
{
    float integral;
    float derivative;
    float prev_error;

    integral = integral + (error * dt);
    
    derivative = (error - prev_error) / dt;

    prev_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    return output;
}

IFX_STATIC void TorqueLimit(void)
{
    float power_over;
    if(RVC.power.value <= POWER_LIMIT)
    {
        power_over = 0;
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
    void PowerComputation();
    void TorqueLimit();
}

